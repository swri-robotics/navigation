#ifndef MOVE_BASE_PATH_MATH_H_
#define MOVE_BASE_PATH_MATH_H_
#include <vector>
#include <geometry_msgs/PoseStamped.h>

namespace move_base
{


double pose_distance(geometry_msgs::PoseStamped& pose1, geometry_msgs::PoseStamped& pose2);
double path_distance(std::vector<geometry_msgs::PoseStamped>* path, int start=0, int end=-1);
int get_closest(geometry_msgs::PoseStamped& pose, std::vector<geometry_msgs::PoseStamped>* path);

};
#endif

