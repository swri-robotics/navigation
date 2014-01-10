#ifndef GLOBAL_NAVIGATOR_H_
#define GLOBAL_NAVIGATOR_H_
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
namespace move_base {

    enum PlannerState { IDLE, PLANNING };
    enum PlanState { NONE, VALID, FAILED }; 

    class GlobalNavigator {
        public: 
              GlobalNavigator(tf::TransformListener& tf); 
              ~GlobalNavigator();
              
            void start(){ planner_costmap_ros_->start(); }
              void stop(){ planner_costmap_ros_->stop(); }
              std::string getGlobalFrameID(){ return planner_costmap_ros_->getGlobalFrameID(); }
              costmap_2d::Costmap2DROS* getCostmap(){ return planner_costmap_ros_; }  
              void setGoal(geometry_msgs::PoseStamped goal);
              PlannerState getPlannerState() { return planner_state_; }
              PlanState getPlanState() { return plan_state_; }
              
              std::vector<geometry_msgs::PoseStamped> getPlan() { 
                boost::unique_lock<boost::mutex> lock(planner_mutex_);
                lock.lock();
                std::vector<geometry_msgs::PoseStamped> return_plan = *planner_plan_; 
                lock.unlock();
                return return_plan;
              }
              
      /**
       * @brief  Make a new global plan
       * @param  goal The goal to plan to
       * @param  plan Will be filled in with the plan made by the planner
       * @return  True if planning succeeds, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
        
        private:
              void planThread();
              tf::TransformListener& tf_;
              boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
              costmap_2d::Costmap2DROS* planner_costmap_ros_;
              pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
              
              std::vector<geometry_msgs::PoseStamped>* planner_plan_;
      std::vector<geometry_msgs::PoseStamped>* latest_plan_;
      
              geometry_msgs::PoseStamped planner_goal_;
            bool p_freq_change_;
      boost::condition_variable planner_cond_;
      ros::Time last_valid_plan_;
              boost::mutex planner_mutex_;
              boost::thread* planner_thread_;
            double planner_frequency_, planner_patience_;
            
            PlannerState planner_state_;
            PlanState plan_state_;
    };

}; // namespace move_base

#endif
