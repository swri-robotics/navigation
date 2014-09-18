#ifndef PATH_ALIGN_COST_FUNCTION_H_
#define PATH_ALIGN_COST_FUNCTION_H_

#include <dwa_local_planner/offset_grid_cost_function.h>

namespace dwa_local_planner {

class PathAlignCostFunction: public OffsetGridCostFunction {
public:
  void initialize(std::string name, base_local_planner::LocalPlannerUtil *planner_util);
  bool prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec);
};

} /* namespace dwa_local_planner */
#endif /* PATH_ALIGN_COST_FUNCTION_H_ */
