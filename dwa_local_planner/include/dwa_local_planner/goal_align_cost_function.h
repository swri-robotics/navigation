#ifndef GOAL_ALIGN_COST_FUNCTION_H_
#define GOAL_ALIGN_COST_FUNCTION_H_

#include <dwa_local_planner/offset_grid_cost_function.h>

namespace dwa_local_planner {

class GoalAlignCostFunction: public OffsetGridCostFunction {
public:
  bool prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec);
};

} /* namespace dwa_local_planner */
#endif /* GOAL_ALIGN_COST_FUNCTION_H_ */
