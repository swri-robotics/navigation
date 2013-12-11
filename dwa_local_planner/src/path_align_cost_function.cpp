#include <dwa_local_planner/path_align_cost_function.h>

using base_local_planner::Trajectory;

namespace dwa_local_planner {

bool PathAlignCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) {
  map_.resetPathDist();
  map_.setTargetCells(*costmap_, target_poses_);
  return true;
}

} /* namespace dwa_local_planner */

 
