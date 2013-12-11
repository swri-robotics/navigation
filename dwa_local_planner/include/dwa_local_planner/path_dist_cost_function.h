#ifndef PATH_DIST_COST_FUNCTION_H_
#define PATH_DIST_COST_FUNCTION_H_

#include <dwa_local_planner/map_grid_cost_function.h>

namespace dwa_local_planner {

class PathDistCostFunction: public MapGridCostFunction {
public:
  bool prepare();
};

} /* namespace dwa_local_planner */
#endif /* PATH_DIST_COST_FUNCTION_H_ */
