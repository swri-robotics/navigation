#ifndef DWA_TRAJECTORYCOSTFUNCTION_H_
#define DWA_TRAJECTORYCOSTFUNCTION_H_

#include <base_local_planner/trajectory.h>
#include <base_local_planner/local_planner_util.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>

namespace dwa_local_planner {

/**
 * @class TrajectoryCostFunction
 * @brief Provides an interface for critics of trajectories
 * During each sampling run, a batch of many trajectories will be scored using such a cost function.
 * The prepare method is called before each batch run, and then for each
 * trajectory of the sampling set, score_trajectory may be called.
 */
class TrajectoryCostFunction {
public:

  void initialize(costmap_2d::Costmap2D* costmap, base_local_planner::LocalPlannerUtil *planner_util, double scale) {
    planner_util_ = planner_util;
    costmap_ = costmap;
    scale_ = scale;
  }

  /**
   * General updating of context values if required.
   * Subclasses may overwrite. Return false in case there is any error.
   */
  virtual bool prepare(tf::Stamped<tf::Pose> global_pose,
      tf::Stamped<tf::Pose> global_vel,
      std::vector<geometry_msgs::Point> footprint_spec) = 0;

  virtual void reset() {}

  /**
   * Set Global Plan with plan transformed and cropped into local costmap frame
   */
  virtual void setGlobalPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan, double goal_x, double goal_y) {}

  /**
   * return a score for trajectory traj
   */
  virtual double scoreTrajectory(base_local_planner::Trajectory &traj) = 0;

  virtual void debrief(base_local_planner::Trajectory &result) {}

  double getScale() {
    return scale_;
  }

  void setScale(double scale) {
    scale_ = scale;
  }

  virtual ~TrajectoryCostFunction() {}

protected:
  base_local_planner::LocalPlannerUtil *planner_util_;
  costmap_2d::Costmap2D* costmap_;
  double scale_;
};

}

#endif /* TRAJECTORYCOSTFUNCTION_H_ */
