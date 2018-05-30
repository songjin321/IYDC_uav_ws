//
// Created by songjin on 18-5-30.
//

#ifndef UAV_CONTROLLER_ROSMATH_H
#define UAV_CONTROLLER_ROSMATH_H

#include <geometry_msgs/PoseStamped.h>

class RosMath {
public:
    static double calDistance(const geometry_msgs::PoseStamped &p1, const geometry_msgs::PoseStamped &p2);
};


#endif //UAV_CONTROLLER_ROSMATH_H
