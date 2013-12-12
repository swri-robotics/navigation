#include <dwa_local_planner/offset_grid_cost_function.h>

using base_local_planner::Trajectory;

namespace dwa_local_planner {

double OffsetGridCostFunction::scoreCell(double px, double py, double pth)
{
    double xshift=xshift_, yshift=yshift_;

    if(scale_offset_>=0.0)
    {
        double d = sqrt(pow(px-goal_x_,2)+pow(py-goal_y_,2));
        double shift_d = sqrt(pow(xshift, 2)+pow(yshift,2));
        if(d<shift_d){
            xshift = d * scale_offset_;
            yshift = d * scale_offset_;
        }
    }

    // translate point forward if specified
    if (xshift != 0.0) {
      px = px + xshift * cos(pth);
      py = py + xshift * sin(pth);
    }
    // translate point sideways if specified
    if (yshift != 0.0) {
      px = px + yshift * cos(pth + M_PI_2);
      py = py + yshift * sin(pth + M_PI_2);
    }

    return MapGridCostFunction::scoreCell(px, py, pth);  
}
} /* namespace dwa_local_planner */

 
