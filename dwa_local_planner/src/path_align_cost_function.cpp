#include <dwa_local_planner/path_align_cost_function.h>

PLUGINLIB_EXPORT_CLASS(dwa_local_planner::PathAlignCostFunction, dwa_local_planner::TrajectoryCostFunction)

using base_local_planner::Trajectory;

namespace dwa_local_planner {


void PathAlignCostFunction::initialize(std::string name, base_local_planner::LocalPlannerUtil *planner_util)
{
    MapGridCostFunction::initialize(name, planner_util);
    stop_on_failure_ = false;

    ros::NodeHandle nh("~/" + name_);
    std::string key;
    if (nh.searchParam("forward_point_distance", key))
    {
        nh.getParam(key, xshift_);
        yshift_ = 0.0;
        shift_d_ = xshift_;
    }else{
        xshift_ = 0.325;
        yshift_ = 0.0;
        shift_d_ = xshift_;
    }

    quit_within_radius_ = true;
}

bool PathAlignCostFunction::prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) {
  map_.resetPathDist();
  map_.setTargetCells(*costmap_, target_poses_);
  return true;
}

} /* namespace dwa_local_planner */

 
