#ifndef PATH_ALIGN_COST_FUNCTION_H_
#define PATH_ALIGN_COST_FUNCTION_H_

#include <dwa_local_planner/offset_grid_cost_function.h>

namespace dwa_local_planner {

class PathAlignCostFunction: public OffsetGridCostFunction {
public:
  bool prepare();
};

} /* namespace dwa_local_planner */
#endif /* PATH_ALIGN_COST_FUNCTION_H_ */
