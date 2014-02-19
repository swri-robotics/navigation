#include<move_base/standard_state_machine.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(move_base::StandardStateMachine, move_base::StateMachine)

namespace move_base
{

StandardStateMachine::StandardStateMachine() :
    recovery_loader_("nav_core", "nav_core::RecoveryBehavior")
{
}

StandardStateMachine::~StandardStateMachine() 
{
    recovery_behaviors_.clear();
}

void StandardStateMachine::initialize(tf::TransformListener* tf, GlobalNavigator* planner, LocalNavigator* controller)
{
    StateMachine::initialize(tf, planner, controller);
    ros::NodeHandle private_nh("~");

    //we'll assume the radius of the robot to be consistent with what's specified for the costmaps
    private_nh.param("local_costmap/circumscribed_radius", base_radius_, 0.46);
    private_nh.param("clearing_radius", clearing_radius_,  base_radius_);

    //load any user specified recovery behaviors, and if that fails load the defaults
    if(!loadRecoveryBehaviors(private_nh)) {
        loadDefaultRecoveryBehaviors();
    }    
    
    dsrv_ = new dynamic_reconfigure::Server<move_base::StandardStateMachineConfig>(ros::NodeHandle("~/standard_state_machine"));
    dynamic_reconfigure::Server<move_base::StandardStateMachineConfig>::CallbackType cb = boost::bind(&StandardStateMachine::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    
    reset();
}

void StandardStateMachine::reconfigureCB(move_base::StandardStateMachineConfig &config, uint32_t level){
    conservative_reset_dist_ = config.conservative_reset_dist;
    recovery_behavior_enabled_ = config.recovery_behavior_enabled;
    clearing_rotation_allowed_ = config.clearing_rotation_allowed;
}

void StandardStateMachine::reset()
{
    //we'll start executing recovery behaviors at the beginning of our list
    recovery_index_ = 0;
    state_ = STANDARD;
    recovery_trigger_ = PLANNING_R;
    controller_->publishZeroVelocity();
    /* TODO:
    
          planner_goal_ = goal;
          runPlanner_ = true;
          planner_cond_.notify_one();
          
                    //make sure to reset our timeouts
          last_valid_control_ = ros::Time::now();
          last_valid_plan_ = ros::Time::now();
          last_oscillation_reset_ = ros::Time::now();

          */

}

void StandardStateMachine::executeCycle(int* status, std::string* message)
{
    if(planner_->getPlanState() == FAILED){
        state_ = RECOVERY;
        recovery_trigger_ = PLANNING_R;
        controller_->publishZeroVelocity();
    }
    else{
        ControllerState cstate = controller_->getState();
        switch(cstate){
          case OSCILLATING:
            state_ = RECOVERY;
            recovery_trigger_ = OSCILLATION_R;
            break;
          case ERROR:
            state_ = RECOVERY;
            recovery_trigger_ = CONTROLLING_R;
            break;
          default:
            state_ = STANDARD;
        }
    }
    
    
      
      if(planner_->hasNewPlan()){
        controller_->setGlobalPlan( planner_->getPlan() );
      }else if(planner_->getPlanState()==FAILED){
        controller_->publishZeroVelocity();
      }
      
      if(controller_->getState()==FINISHED){
       reset();
       *status = 1;
       *message = "Goal Reached Motherfuckers!";
       return;
      }
    
    
    if(state_==STANDARD)
    {
        *status = 0;
        return;
    }
        
    ROS_DEBUG_NAMED("move_base","In clearing/recovery state");
    
    //we'll invoke whatever recovery behavior we're currently on if they're enabled
    if(recovery_index_ < recovery_behaviors_.size()){
      ROS_DEBUG_NAMED("move_base_recovery","Executing behavior %u of %zu", recovery_index_, recovery_behaviors_.size());
      recovery_behaviors_[recovery_index_]->runBehavior();

      //we'll check if the recovery behavior actually worked
      //ROS_DEBUG_NAMED("move_base_recovery","Going back to planning state");
      //state_ = PLANNING;

      //update the index of the next recovery behavior that we'll try
      recovery_index_++;
      *status = 0;
    }
    else{
      ROS_DEBUG_NAMED("move_base_recovery","All recovery behaviors have failed, locking the planner and disabling it.");

      if(recovery_trigger_ == CONTROLLING_R){
        ROS_ERROR("Aborting because a valid control could not be found. Even after executing all recovery behaviors");
        *status = -1;
        *message = "Failed to find a valid control. Even after executing recovery behaviors.";
      }
      else if(recovery_trigger_ == PLANNING_R){
        ROS_ERROR("Aborting because a valid plan could not be found. Even after executing all recovery behaviors");
        *status = -1;
        *message ="Failed to find a valid plan. Even after executing recovery behaviors.";
      }
      else if(recovery_trigger_ == OSCILLATION_R){
        ROS_ERROR("Aborting because the robot appears to be oscillating over and over. Even after executing all recovery behaviors");
        *status = -1;
        *message ="Robot is oscillating. Even after executing recovery behaviors.";
      }
    }
}

bool StandardStateMachine::loadRecoveryBehaviors(ros::NodeHandle node) {
    XmlRpc::XmlRpcValue behavior_list;
    if(node.getParam("recovery_behaviors", behavior_list)) {
        if(behavior_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            for(int i = 0; i < behavior_list.size(); ++i) {
                if(behavior_list[i].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                    if(behavior_list[i].hasMember("name") && behavior_list[i].hasMember("type")) {
                        //check for recovery behaviors with the same name
                        for(int j = i + 1; j < behavior_list.size(); j++) {
                            if(behavior_list[j].getType() == XmlRpc::XmlRpcValue::TypeStruct) {
                                if(behavior_list[j].hasMember("name") && behavior_list[j].hasMember("type")) {
                                    std::string name_i = behavior_list[i]["name"];
                                    std::string name_j = behavior_list[j]["name"];
                                    if(name_i == name_j) {
                                        ROS_ERROR("A recovery behavior with the name %s already exists, this is not allowed. Using the default recovery behaviors instead.",
                                                  name_i.c_str());
                                        return false;
                                    }
                                }
                            }
                        }
                    } else {
                        ROS_ERROR("Recovery behaviors must have a name and a type and this does not. Using the default recovery behaviors instead.");
                        return false;
                    }
                } else {
                    ROS_ERROR("Recovery behaviors must be specified as maps, but they are XmlRpcType %d. We'll use the default recovery behaviors instead.",
                              behavior_list[i].getType());
                    return false;
                }
            }

            //if we've made it to this point, we know that the list is legal so we'll create all the recovery behaviors
            for(int i = 0; i < behavior_list.size(); ++i) {
                try {
                    //check if a non fully qualified name has potentially been passed in
                    if(!recovery_loader_.isClassAvailable(behavior_list[i]["type"])) {
                        std::vector<std::string> classes = recovery_loader_.getDeclaredClasses();
                        for(unsigned int i = 0; i < classes.size(); ++i) {
                            if(behavior_list[i]["type"] == recovery_loader_.getName(classes[i])) {
                                //if we've found a match... we'll get the fully qualified name and break out of the loop
                                ROS_WARN("Recovery behavior specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                                         std::string(behavior_list[i]["type"]).c_str(), classes[i].c_str());
                                behavior_list[i]["type"] = classes[i];
                                break;
                            }
                        }
                    }

                    boost::shared_ptr<nav_core::RecoveryBehavior> behavior(recovery_loader_.createInstance(behavior_list[i]["type"]));

                    //shouldn't be possible, but it won't hurt to check
                    if(behavior.get() == NULL) {
                        ROS_ERROR("The ClassLoader returned a null pointer without throwing an exception. This should not happen");
                        return false;
                    }

                    //initialize the recovery behavior with its name
                    behavior->initialize(behavior_list[i]["name"], tf_, planner_->getCostmap(), controller_->getCostmap());
                    recovery_behaviors_.push_back(behavior);
                } catch(pluginlib::PluginlibException& ex) {
                    ROS_ERROR("Failed to load a plugin. Using default recovery behaviors. Error: %s", ex.what());
                    return false;
                }
            }
        } else {
            ROS_ERROR("The recovery behavior specification must be a list, but is of XmlRpcType %d. We'll use the default recovery behaviors instead.",
                      behavior_list.getType());
            return false;
        }
    } else {
        //if no recovery_behaviors are specified, we'll just load the defaults
        return false;
    }

    //if we've made it here... we've constructed a recovery behavior list successfully
    return true;
}

//we'll load our default recovery behaviors here
void StandardStateMachine::loadDefaultRecoveryBehaviors() {
    recovery_behaviors_.clear();
    costmap_2d::Costmap2DROS* planner_costmap_ros = planner_->getCostmap(), *controller_costmap_ros = controller_->getCostmap();
    
    
    try {
        //we need to set some parameters based on what's been passed in to us to maintain backwards compatibility
        ros::NodeHandle n("~");
        n.setParam("conservative_reset/reset_distance", conservative_reset_dist_);
        n.setParam("aggressive_reset/reset_distance", base_radius_ * 4);

        //first, we'll load a recovery behavior to clear the costmap
        boost::shared_ptr<nav_core::RecoveryBehavior> cons_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
        cons_clear->initialize("conservative_reset", tf_, planner_costmap_ros, controller_costmap_ros);
        recovery_behaviors_.push_back(cons_clear);

        //next, we'll load a recovery behavior to rotate in place
        boost::shared_ptr<nav_core::RecoveryBehavior> rotate(recovery_loader_.createInstance("rotate_recovery/RotateRecovery"));
        if(clearing_rotation_allowed_) {
            rotate->initialize("rotate_recovery", tf_, planner_costmap_ros, controller_costmap_ros);
            recovery_behaviors_.push_back(rotate);
        }

        //next, we'll load a recovery behavior that will do an aggressive reset of the costmap
        boost::shared_ptr<nav_core::RecoveryBehavior> ags_clear(recovery_loader_.createInstance("clear_costmap_recovery/ClearCostmapRecovery"));
        ags_clear->initialize("aggressive_reset", tf_, planner_costmap_ros, controller_costmap_ros);
        recovery_behaviors_.push_back(ags_clear);

        //we'll rotate in-place one more time
        if(clearing_rotation_allowed_)
            recovery_behaviors_.push_back(rotate);
    } catch(pluginlib::PluginlibException& ex) {
        ROS_FATAL("Failed to load a plugin. This should not happen on default recovery behaviors. Error: %s", ex.what());
    }

    return;
}

};
