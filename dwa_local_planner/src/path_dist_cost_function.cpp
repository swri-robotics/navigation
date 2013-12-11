#include <dwa_local_planner/path_dist_cost_function.h>

using base_local_planner::Trajectory;

namespace dwa_local_planner {

bool PathDistCostFunction::prepare() {
  map_.resetPathDist();
  map_.setTargetCells(*costmap_, target_poses_);
  return true;
}

} /* namespace dwa_local_planner */

 
