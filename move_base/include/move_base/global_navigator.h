#ifndef GLOBAL_NAVIGATOR_H_
#define GLOBAL_NAVIGATOR_H_
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
namespace move_base {
    class GlobalNavigator {
        public: 
              GlobalNavigator(tf::TransformListener& tf); 
              ~GlobalNavigator();
      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
        
        private:
              tf::TransformListener& tf_;
              boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
              costmap_2d::Costmap2DROS* planner_costmap_ros_;
              pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
              std::vector<geometry_msgs::PoseStamped>* planner_plan_;
    };

}; // namespace move_base

#endif
