#ifndef LOCAL_NAVIGATOR_H_
#define LOCAL_NAVIGATOR_H_
#include <nav_core/base_local_planner.h>
#include <costmap_2d/costmap_2d_ros.h>

enum ControllerState { IDLE, CONTROLLING, OSCILLATING, ERROR, INVALID_CONTROL, FINISHED };

namespace move_base {
class LocalNavigator {
    public:
        LocalNavigator(tf::TransformListener& tf);
        ~LocalNavigator();

        void start() {
            controller_costmap_ros_->start();
        }
        void stop() {
            controller_costmap_ros_->stop();
        }
        costmap_2d::Costmap2DROS* getCostmap() {
            return controller_costmap_ros_;
        }

		ControllerState getState() { return state_; }

        void setGlobalPlan( std::vector<geometry_msgs::PoseStamped> global_plan );

        /**
         * @brief  Publishes a velocity command of zero to the base
         */
        void publishZeroVelocity();

    private:
        void controlThread();

        ControllerState state_;
        double controller_frequency_, controller_patience_;
        boost::thread* control_thread_;
        ros::Publisher vel_pub_;
        tf::TransformListener& tf_;
        boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
        costmap_2d::Costmap2DROS* controller_costmap_ros_;
        pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
        std::vector<geometry_msgs::PoseStamped> controller_plan_;
        bool c_freq_change_;


  double oscillation_timeout_, oscillation_distance_;
  ros::Time last_valid_control_, last_oscillation_reset_;
  geometry_msgs::PoseStamped oscillation_pose_;
};

}; // namespace move_base

#endif

