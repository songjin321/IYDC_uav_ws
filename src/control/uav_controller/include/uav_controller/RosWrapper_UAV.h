//
// Created by songjin on 18-5-25.
//

#ifndef UAV_CONTROLLER_ROSWRAPPER_UAV_H
#define UAV_CONTROLLER_ROSWRAPPER_UAV_H

#include "nav_msgs/GetPlan.h"
#include "uav_controller/UnmannedAerialVehicle.h"
#include <ros/ros.h>
class RosWrapper_UAV
{
public:
    RosWrapper_UAV(std::string vision_pose_name, std::string path_planner_name, UnmannedAerialVehicle* uav);
    void vision_pose_callback(const geometry_msgs::PoseStamped &vision_pose);
    bool fly_to_goal(const geometry_msgs::PoseStamped &goal_pose);
private:
    UnmannedAerialVehicle *p_uav_;
    ros::NodeHandle n_;
    ros::Subscriber vision_pose_sub_;
    ros::Publisher mavros_set_point_pub_;
    ros::ServiceClient planner_client_;

};


#endif //UAV_CONTROLLER_ROSWRAPPER_UAV_H
