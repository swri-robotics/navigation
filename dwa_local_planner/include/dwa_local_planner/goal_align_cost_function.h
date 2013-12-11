#ifndef GOAL_ALIGN_COST_FUNCTION_H_
#define GOAL_ALIGN_COST_FUNCTION_H_

#include <dwa_local_planner/offset_grid_cost_function.h>

namespace dwa_local_planner {

class GoalAlignCostFunction: public OffsetGridCostFunction {
public:
  bool prepare();
};

} /* namespace dwa_local_planner */
#endif /* GOAL_ALIGN_COST_FUNCTION_H_ */
