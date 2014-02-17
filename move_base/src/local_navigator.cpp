#include<move_base/local_navigator.h>
#include<move_base/utils.h>

namespace move_base {

LocalNavigator::LocalNavigator(tf::TransformListener& tf) :
    tf_(tf), controller_costmap_ros_(NULL), c_freq_change_(false),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner") {
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    std::string local_planner;
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));


    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("controller_patience", controller_patience_, 15.0);

    private_nh.param("oscillation_timeout", oscillation_timeout_, 0.0);
    private_nh.param("oscillation_distance", oscillation_distance_, 0.5);

    //create the ros wrapper for the controller's costmap... and initializer a pointer we'll use with the underlying map
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    controller_costmap_ros_->pause();

    //create a local planner
    try {
        //check if a non fully qualified name has potentially been passed in
        if(!blp_loader_.isClassAvailable(local_planner)) {
            std::vector<std::string> classes = blp_loader_.getDeclaredClasses();
            for(unsigned int i = 0; i < classes.size(); ++i) {
                if(local_planner == blp_loader_.getName(classes[i])) {
                    //if we've found a match... we'll get the fully qualified name and break out of the loop
                    ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                             local_planner.c_str(), classes[i].c_str());
                    local_planner = classes[i];
                    break;
                }
            }
        }

        tc_ = blp_loader_.createInstance(local_planner);
        tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
        exit(1);
    }

    //for comanding the base
    vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    controller_costmap_ros_->start();
    
    state_ = IDLE;
    //set up the planner's thread
    control_thread_ = new boost::thread(boost::bind(&LocalNavigator::controlThread, this));

}

LocalNavigator::~LocalNavigator() {
    if(controller_costmap_ros_ != NULL)
        delete controller_costmap_ros_;
    delete control_thread_;
}

void LocalNavigator::publishZeroVelocity() {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
}

void LocalNavigator::setGlobalPlan( std::vector<geometry_msgs::PoseStamped> global_plan ) {
    controller_plan_ = global_plan;
    if(tc_->setPlan(controller_plan_)){
        state_ = CONTROLLING;
    }else{
        state_ = ERROR;
    }    
}

void LocalNavigator::controlThread() {
    ros::NodeHandle n;
    geometry_msgs::Twist cmd_vel;
    ros::Rate r(controller_frequency_);

    while(n.ok()) {
        //for timing that gives real time even in simulation
        ros::WallTime start = ros::WallTime::now();


        if(c_freq_change_) {
            ROS_INFO("Setting controller frequency to %.2f", controller_frequency_);
            r = ros::Rate(controller_frequency_);
            c_freq_change_ = false;
        }


        // check oscillation progress
        tf::Stamped<tf::Pose> global_pose;
        controller_costmap_ros_->getRobotPose(global_pose);
        geometry_msgs::PoseStamped current_position;
        tf::poseStampedTFToMsg(global_pose, current_position);

        //check to see if we've moved far enough to reset our oscillation timeout
        if(pose_distance(current_position, oscillation_pose_) >= oscillation_distance_)
        {
          last_oscillation_reset_ = ros::Time::now();
          oscillation_pose_ = current_position;
          // TODO: reset?  oscillation
        }


        if(state_ == CONTROLLING || state_ == INVALID_CONTROL)
        {
            //check that the observation buffers for the costmap are current, we don't want to drive blind
            if(!controller_costmap_ros_->isCurrent()) {
                ROS_WARN("[%s]:Sensor data is out of date, we're not going to allow commanding of the base for safety",ros::this_node::getName().c_str());
                publishZeroVelocity();
                state_ = ERROR;
                continue;
            } 

            //check to see if we've reached our goal
            if(tc_->isGoalReached()) {
                ROS_DEBUG_NAMED("move_base","Goal reached!");
                state_ = FINISHED;
                continue;
            }

		    //check for an oscillation condition
		    if(oscillation_timeout_ > 0.0 &&
		            last_oscillation_reset_ + ros::Duration(oscillation_timeout_) < ros::Time::now()) {
		        publishZeroVelocity();
		        state_ = OSCILLATING;
		    }

			// Actual Control
            {
                boost::unique_lock< boost::shared_mutex > lock(*(controller_costmap_ros_->getCostmap()->getLock()));

                if(tc_->computeVelocityCommands(cmd_vel)) {
                    ROS_DEBUG_NAMED("move_base", "Got a valid command from the local planner.");
                    last_valid_control_ = ros::Time::now();
                    //make sure that we send the velocity command to the base
                    vel_pub_.publish(cmd_vel);
                } else {
                    ROS_DEBUG_NAMED("move_base", "The local planner could not find a valid plan.");
                    ros::Time attempt_end = last_valid_control_ + ros::Duration(controller_patience_);

                    //check if we've tried to find a valid control for longer than our time limit
                    if(ros::Time::now() > attempt_end) {
                        //we'll move into our obstacle clearing mode
                        publishZeroVelocity();
                        state_ = ERROR;
                    } else {
                        state_ = INVALID_CONTROL;
                    }
                }
            }

        }
        

        ros::WallDuration t_diff = ros::WallTime::now() - start;
        ROS_DEBUG_NAMED("move_base","Full control cycle time: %.9f\n", t_diff.toSec());

        r.sleep();
        //make sure to sleep for the remainder of our cycle time
        if(r.cycleTime() > ros::Duration(1 / controller_frequency_) && state_ == CONTROLLING)
            ROS_WARN("Control loop missed its desired rate of %.4fHz... the loop actually took %.4f seconds", controller_frequency_, r.cycleTime().toSec());
    }

}

}; // namespace move_base
