#ifndef _MOVE_BASE_UTILS_
#define _MOVE_BASE_UTILS_

double pose_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
    return sqrt((p1.pose.position.x - p2.pose.position.x) * (p1.pose.position.x - p2.pose.position.x)
                + (p1.pose.position.y - p2.pose.position.y) * (p1.pose.position.y - p2.pose.position.y));
}

#endif
