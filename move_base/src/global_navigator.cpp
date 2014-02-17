#include<move_base/global_navigator.h>

namespace move_base {

GlobalNavigator::GlobalNavigator(tf::TransformListener& tf) :
    tf_(tf),
    planner_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner"), p_freq_change_(false) {
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    std::string global_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    private_nh.param("planner_frequency", planner_frequency_, 0.0);
    private_nh.param("planner_patience", planner_patience_, 5.0);
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    planner_state_ = IDLE;
    plan_state_ = NONE;
    has_new_plan_ = false;

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    try {
        //check if a non fully qualified name has potentially been passed in
        if(!bgp_loader_.isClassAvailable(global_planner)) {
            std::vector<std::string> classes = bgp_loader_.getDeclaredClasses();
            for(unsigned int i = 0; i < classes.size(); ++i) {
                if(global_planner == bgp_loader_.getName(classes[i])) {
                    //if we've found a match... we'll get the fully qualified name and break out of the loop
                    ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                             global_planner.c_str(), classes[i].c_str());
                    global_planner = classes[i];
                    break;
                }
            }
        }

        planner_ = bgp_loader_.createInstance(global_planner);
        planner_->initialize(bgp_loader_.getName(global_planner), planner_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
        exit(1);
    }

    planner_costmap_ros_->start();

    //set up the planner's thread
    planner_thread_ = new boost::thread(boost::bind(&GlobalNavigator::planThread, this));
}

GlobalNavigator::~GlobalNavigator() {
    if(planner_costmap_ros_ != NULL)
        delete planner_costmap_ros_;

    planner_thread_->interrupt();
    planner_thread_->join();
    delete planner_thread_;
    delete planner_plan_;
}

void GlobalNavigator::setGoal(geometry_msgs::PoseStamped goal) {
    planner_goal_ = goal;
    replan();
}

void GlobalNavigator::replan(){
    plan_state_ = NONE;
    planner_state_ = PLANNING;
    has_new_plan_ = false;
    planner_cond_.notify_one();
    
}


void GlobalNavigator::planThread() {
    ROS_DEBUG_NAMED("move_base_plan_thread","Starting planner thread...");
    ros::NodeHandle n;
    ros::Rate r(planner_frequency_);
    boost::unique_lock<boost::mutex> lock(planner_mutex_);
    while(n.ok()) {
        if(p_freq_change_) {
            ROS_INFO("Setting planner frequency to %.2f", planner_frequency_);
            r = ros::Rate(planner_frequency_);
            p_freq_change_ = false;
        }

        //check if we should run the planner (the mutex is locked)
        while(planner_state_ != PLANNING) {
            //if we should not be running the planner then suspend this thread
            ROS_DEBUG_NAMED("move_base_plan_thread","Planner thread is suspending");
            planner_cond_.wait(lock);
        }

        //time to plan! get a copy of the goal and unlock the mutex
        geometry_msgs::PoseStamped temp_goal = planner_goal_;
        lock.unlock();
        ROS_DEBUG_NAMED("move_base_plan_thread","Planning...");

        //run planner
        planner_plan_->clear();
        bool gotPlan = n.ok() && makePlan(temp_goal, *planner_plan_);

        if(gotPlan) {
            ROS_DEBUG_NAMED("move_base_plan_thread","Got Plan with %zu points!", planner_plan_->size());
            //pointer swap the plans under mutex (the controller will pull from latest_plan_)
            std::vector<geometry_msgs::PoseStamped>* temp_plan = planner_plan_;

            lock.lock();
            has_new_plan_ = true;
            plan_state_ = VALID;
            planner_plan_ = latest_plan_;
            latest_plan_ = temp_plan;
            last_valid_plan_ = ros::Time::now();

            ROS_DEBUG_NAMED("move_base_plan_thread","Generated a plan from the base_global_planner");

            if(planner_frequency_ <= 0)
                planner_state_ = IDLE;
            lock.unlock();
        }
        //if we didn't get a plan and we are in the planning state (the robot isn't moving)
        else if(plan_state_ == NONE) {
            ROS_DEBUG_NAMED("move_base_plan_thread","No Plan...");
            ros::Time attempt_end = last_valid_plan_ + ros::Duration(planner_patience_);

            //check if we've tried to make a plan for over our time limit
            if(ros::Time::now() > attempt_end) {
                //we'll move into our obstacle clearing mode
                planner_state_ = IDLE;
                plan_state_ = FAILED;
            }
        }

        if(!p_freq_change_ && planner_frequency_ > 0)
            r.sleep();

        //take the mutex for the next iteration
        lock.lock();
    }
}

///TODO:          local_nav_.publishZeroVelocity();


bool GlobalNavigator::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan) {
    boost::unique_lock< boost::shared_mutex > lock(*(planner_costmap_ros_->getCostmap()->getLock()));

    //make sure to set the plan to be empty initially
    plan.clear();

    //since this gets called on handle activate
    if(planner_costmap_ros_ == NULL) {
        ROS_ERROR("Planner costmap ROS is NULL, unable to create global plan");
        return false;
    }

    //get the starting pose of the robot
    tf::Stamped<tf::Pose> global_pose;
    if(!planner_costmap_ros_->getRobotPose(global_pose)) {
        ROS_WARN("Unable to get starting pose of robot, unable to create global plan");
        return false;
    }

    geometry_msgs::PoseStamped start;
    tf::poseStampedTFToMsg(global_pose, start);

    //if the planner fails or returns a zero length plan, planning failed
    if(!planner_->makePlan(start, goal, plan) || plan.empty()) {
        ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
        return false;
    }

    return true;
}




}; // namespace move_base
