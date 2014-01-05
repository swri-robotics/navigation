#include <dwa_local_planner/dwa_planner_ros.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "standalone");

    tf::TransformListener tf;

    dwa_local_planner::DWAPlannerROS planner;
    costmap_2d::Costmap2DROS costmap("local", tf);
    planner.initialize("planner", &tf, &costmap);   

    std::vector<geometry_msgs::PoseStamped> plan;
    for(int i=0;i<100;i++){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/map";
        pose.pose.orientation.w = 1.0;
        pose.pose.position.y = i / 50.;
        plan.push_back(pose);
    }
    planner.setPlan(plan);

    geometry_msgs::Twist cmd_vel;
    planner.computeVelocityCommands(cmd_vel);
    ROS_INFO("%f %f %f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
 
    return 0;
}
