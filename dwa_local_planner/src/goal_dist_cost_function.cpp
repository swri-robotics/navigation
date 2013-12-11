#include <dwa_local_planner/goal_dist_cost_function.h>

using base_local_planner::Trajectory;

namespace dwa_local_planner {

bool GoalDistCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) {
  map_.resetPathDist();
  map_.setLocalGoal(*costmap_, target_poses_);
  return true;
}
} /* namespace dwa_local_planner */

 
