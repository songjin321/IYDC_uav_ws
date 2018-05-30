//
// Created by songjin on 18-5-30.
//

#include "ros_common/RosMath.h"
#include <geometry_msgs/PoseStamped.h>
double RosMath::calDistance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2)
{
    double dx = p1.pose.position.x - p2.pose.position.x;
    double dy = p1.pose.position.y - p2.pose.position.y;
    double dz = p1.pose.position.z - p2.pose.position.z;
    return sqrt(dx*dx + dy*dy + dz*dz);
}