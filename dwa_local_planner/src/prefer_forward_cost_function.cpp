/*
 * prefer_forward_cost_function.cpp
 *
 *  Created on: Apr 4, 2012
 *      Author: tkruse
 */

#include <dwa_local_planner/prefer_forward_cost_function.h>

#include <math.h>

PLUGINLIB_EXPORT_CLASS(dwa_local_planner::PreferForwardCostFunction, dwa_local_planner::TrajectoryCostFunction)


using base_local_planner::Trajectory;

namespace dwa_local_planner {

void PreferForwardCostFunction::initialize(std::string name, base_local_planner::LocalPlannerUtil *planner_util)
{
    TrajectoryCostFunction::initialize(name, planner_util);
    penalty_ = 1.0;
}


double PreferForwardCostFunction::scoreTrajectory(Trajectory &traj) {
  // backward motions bad on a robot without backward sensors
  if (traj.xv_ < 0.0) {
    return penalty_;
  }
  // strafing motions also bad on such a robot
  if (traj.xv_ < 0.1 && fabs(traj.thetav_) < 0.2) {
    return penalty_;
  }
  // the more we rotate, the less we progress forward
  return fabs(traj.thetav_) * 10;
}

} /* namespace dwa_local_planner */
