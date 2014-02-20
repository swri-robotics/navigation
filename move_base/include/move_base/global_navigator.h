#ifndef GLOBAL_NAVIGATOR_H_
#define GLOBAL_NAVIGATOR_H_
#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_msgs/GetPlan.h>
#include <move_base/GlobalNavConfig.h>

namespace move_base {

enum PlannerState { IDLE, PLANNING };
enum PlanState { NONE, VALID, FAILED };

class GlobalNavigator {
    public:
        GlobalNavigator(tf::TransformListener& tf);
        ~GlobalNavigator();

        void start() {
            planner_costmap_ros_->start();
        }
        void stop() {
            planner_costmap_ros_->stop();
            replan();
        }
        std::string getGlobalFrameID() {
            return planner_costmap_ros_->getGlobalFrameID();
        }
        costmap_2d::Costmap2DROS* getCostmap() {
            return planner_costmap_ros_;
        }
        
        PlannerState getPlannerState() {
            return planner_state_;
        }
        PlanState getPlanState() {
            return plan_state_;
        }
        bool hasNewPlan(){
            boost::unique_lock<boost::mutex> lock(planner_mutex_);
            if(plan_state_==VALID and has_new_plan_){
                has_new_plan_ = false;
                return true;
            }
            return false;
        }

        std::vector<geometry_msgs::PoseStamped> getPlan() {
            boost::unique_lock<boost::mutex> lock(planner_mutex_);
            std::vector<geometry_msgs::PoseStamped> return_plan = *latest_plan_;
            return return_plan;
        }


        void updatePosition(geometry_msgs::PoseStamped pos);
        void setGoal(geometry_msgs::PoseStamped goal);
        void replan(bool reset_values=false);

        /**
         * @brief  Make a new global plan
         * @param  goal The goal to plan to
         * @param  plan Will be filled in with the plan made by the planner
         * @return  True if planning succeeds, false otherwise
         */
        bool makePlan(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);
        
         /**
           * @brief  A service call that can be made when the action is inactive that will return a plan
           * @param  req The goal request
           * @param  resp The plan request
           * @return True if planning succeeded, false otherwise
           */
        bool planService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &resp);
        
        double getDistanceTravelled(){ return distance_travelled_; }
        double getDistanceLeft(){ return distance_left_; }
        double getPercentComplete(){ return distance_travelled_ / (distance_travelled_ + distance_left_); }

    private:
        void planThread();
        tf::TransformListener& tf_;
        boost::shared_ptr<nav_core::BaseGlobalPlanner> planner_;
        costmap_2d::Costmap2DROS* planner_costmap_ros_;
        pluginlib::ClassLoader<nav_core::BaseGlobalPlanner> bgp_loader_;
        ros::ServiceServer make_plan_srv_;

        std::vector<geometry_msgs::PoseStamped>* planner_plan_;
        std::vector<geometry_msgs::PoseStamped>* latest_plan_;

        geometry_msgs::PoseStamped current_pos_, prev_, planner_goal_;
        bool p_freq_change_;
        boost::condition_variable planner_cond_;
        ros::Time last_valid_plan_;
        boost::mutex planner_mutex_;
        boost::thread* planner_thread_;
        double planner_frequency_, planner_patience_;
        
        double distance_travelled_, distance_left_;

        PlannerState planner_state_;
        PlanState plan_state_;
        bool has_new_plan_;

        dynamic_reconfigure::Server<move_base::GlobalNavConfig> *dsrv_;
        void reconfigureCB(move_base::GlobalNavConfig &config, uint32_t level);

};

}; // namespace move_base

#endif

