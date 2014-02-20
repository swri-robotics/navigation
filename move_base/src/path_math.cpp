#include<move_base/path_math.h>

namespace move_base
{

double pose_distance(geometry_msgs::PoseStamped& pose1, geometry_msgs::PoseStamped& pose2)
{
    double dx = pose1.pose.position.x - pose2.pose.position.x,
           dy = pose1.pose.position.y - pose2.pose.position.y;
    return sqrt(dx*dx + dy*dy);
}

double path_distance(std::vector<geometry_msgs::PoseStamped>* path, int start, int end)
{
    double total = 0.0;
    
    if(end<0)
        end = path->size()-1;
    
    for(int i=start; i<end; i++){
        total += pose_distance( (*path)[i], (*path)[i+1] );
    }
    
    return total;
}

int get_closest(geometry_msgs::PoseStamped& pose, std::vector<geometry_msgs::PoseStamped>* path)
{
    int best_i = -1;
    double best_d = 0.0;
    
    for(int i=0;i<path->size();i++)
    {
        double d = pose_distance(pose, (*path)[i]);
        if(best_i == -1 or best_d > d){
            best_i = i;
            best_d = d;
        }
    }
    
    return best_i;
}

};
