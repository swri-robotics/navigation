#ifndef OFFSET_GRID_COST_FUNCTION_H_
#define OFFSET_GRID_COST_FUNCTION_H_

#include <dwa_local_planner/map_grid_cost_function.h>

namespace dwa_local_planner {

class OffsetGridCostFunction: public MapGridCostFunction {
public:

  virtual bool prepare(tf::Stamped<tf::Pose> global_pose,
		       tf::Stamped<tf::Pose> global_vel,
		       std::vector<geometry_msgs::Point> footprint_spec) = 0;
  virtual double scoreCell(double px, double py, double pth);
protected:
    double xshift_, yshift_, scale_offset_;
};

} /* namespace dwa_local_planner */
#endif /* OFFSET_GRID_COST_FUNCTION_H_ */
