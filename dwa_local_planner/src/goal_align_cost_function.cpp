#include <dwa_local_planner/goal_align_cost_function.h>

using base_local_planner::Trajectory;

namespace dwa_local_planner {

bool GoalAlignCostFunction::prepare() {
  map_.resetPathDist();
  map_.setLocalGoal(*costmap_, target_poses_);
  return true;
}
} /* namespace dwa_local_planner */

 
