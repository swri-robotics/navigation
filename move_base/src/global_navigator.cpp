#include<move_base/global_navigator.h>

namespace move_base {

GlobalNavigator::GlobalNavigator(tf::TransformListener& tf) :
     tf_(tf),
    planner_costmap_ros_(NULL),
    bgp_loader_("nav_core", "nav_core::BaseGlobalPlanner")
{
    ros::NodeHandle private_nh("~");
    ros::NodeHandle nh;
    std::string global_planner;
    private_nh.param("base_global_planner", global_planner, std::string("navfn/NavfnROS"));
    planner_plan_ = new std::vector<geometry_msgs::PoseStamped>();

    //create the ros wrapper for the planner's costmap... and initializer a pointer we'll use with the underlying map
    planner_costmap_ros_ = new costmap_2d::Costmap2DROS("global_costmap", tf_);
    planner_costmap_ros_->pause();

    //initialize the global planner
    try {
      //check if a non fully qualified name has potentially been passed in
      if(!bgp_loader_.isClassAvailable(global_planner)){
        std::vector<std::string> classes = bgp_loader_.getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i){
          if(global_planner == bgp_loader_.getName(classes[i])){
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
    } catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", global_planner.c_str(), ex.what());
      exit(1);
    }
    
    planner_costmap_ros_->start();
}     

GlobalNavigator::~GlobalNavigator(){
    if(planner_costmap_ros_ != NULL)
      delete planner_costmap_ros_;
}

bool GlobalNavigator::makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan){
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
    if(!planner_->makePlan(start, goal, plan) || plan.empty()){
      ROS_DEBUG_NAMED("move_base","Failed to find a  plan to point (%.2f, %.2f)", goal.pose.position.x, goal.pose.position.y);
      return false;
    }

    return true;
}




}; // namespace move_base
