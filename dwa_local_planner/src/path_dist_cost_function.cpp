#include <dwa_local_planner/path_dist_cost_function.h>

PLUGINLIB_EXPORT_CLASS(dwa_local_planner::PathDistCostFunction, dwa_local_planner::TrajectoryCostFunction)

using base_local_planner::Trajectory;

namespace dwa_local_planner {

bool PathDistCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) {
  map_.resetPathDist();
  map_.setTargetCells(*costmap_, target_poses_);
  return true;
}

} /* namespace dwa_local_planner */

 
