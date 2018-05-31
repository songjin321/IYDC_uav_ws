//
// Created by songjin on 18-5-25.
//

#ifndef UAV_CONTROLLER_ROSWRAPPERUAV_H
#define UAV_CONTROLLER_ROSWRAPPERUAV_H

#include "nav_msgs/GetPlan.h"
#include "uav_controller/RosWrapperUAV.h"
#include <ros/ros.h>
/*
 * 保存无人机的状态,利用mavros底层控制无人机运动
 * 利用glog,将飞行状态记录下来
 */
class RosWrapperUAV
{
public:
    RosWrapperUAV(std::string vision_pose_name);
    void vision_pose_callback(const geometry_msgs::PoseStamped &vision_pose);
    void fly_to_goal(const geometry_msgs::PoseStamped &goal_pose, double fly_vel);
    geometry_msgs::PoseStamped getCurrentPoseStamped();
private:
    ros::NodeHandle n_;
    ros::Subscriber vision_pose_sub_;
    ros::Publisher mavros_set_point_pub_;
    geometry_msgs::PoseStamped current_pose_;
};

#endif