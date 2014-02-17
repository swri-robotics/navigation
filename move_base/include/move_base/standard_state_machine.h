#ifndef MOVE_BASE_STANDARD_STATE_MACHINE_
#define MOVE_BASE_STANDARD_STATE_MACHINE_

#include<move_base/state_machine.h>
#include<nav_core/recovery_behavior.h>

enum SState { STANDARD, RECOVERY };
enum RecoveryTrigger
{
  PLANNING_R, CONTROLLING_R, OSCILLATION_R
};

namespace move_base {

  class StandardStateMachine : public StateMachine {
    public:
      StandardStateMachine();
      ~StandardStateMachine();
      
      virtual void initialize(tf::TransformListener* tf, GlobalNavigator* planner, LocalNavigator* controller);

      virtual void reset();
      virtual std::string executeCycle();

    private:
      SState state_;


  /**
   * @brief  Load the recovery behaviors for the navigation stack from the parameter server
   * @param node The ros::NodeHandle to be used for loading parameters
   * @return True if the recovery behaviors were loaded successfully, false otherwise
   */
  bool loadRecoveryBehaviors(ros::NodeHandle node);

  /**
   * @brief  Loads the default recovery behaviors for the navigation stack
   */
  void loadDefaultRecoveryBehaviors();

  RecoveryTrigger recovery_trigger_;
  pluginlib::ClassLoader<nav_core::RecoveryBehavior> recovery_loader_;
  std::vector<boost::shared_ptr<nav_core::RecoveryBehavior> > recovery_behaviors_;
  unsigned int recovery_index_;
  bool clearing_rotation_allowed_, recovery_behavior_enabled_;
  double base_radius_, conservative_reset_dist_, clearing_radius_;
  };
};

#endif
