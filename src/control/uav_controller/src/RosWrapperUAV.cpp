//
// Created by songjin on 18-5-25.
//

#include "uav_controller/RosWrapperUAV.h"
#include "nav_msgs/GetPlan.h"
#include "ros_common/RosMath.h"
#include <iostream>
RosWrapperUAV::RosWrapperUAV(std::string vision_pose_name)
{
    vision_pose_sub_ = n_.subscribe(vision_pose_name, 1, &RosWrapperUAV::vision_pose_callback, this);
    mavros_set_point_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",1);
    current_pose_.pose.orientation.w = 1.0;
}

void RosWrapperUAV::vision_pose_callback(const geometry_msgs::PoseStamped &vision_pose)
{
    current_pose_ =  vision_pose;
}
geometry_msgs::PoseStamped RosWrapperUAV::getCurrentPoseStamped() {
    return current_pose_;
}
void RosWrapperUAV::fly_to_goal(const geometry_msgs::PoseStamped &goal_pose, double fly_vel)
{

    if (fly_vel <= 0)
    {
        mavros_set_point_pub_.publish(goal_pose);
    } else {
        // mavros fly using velocity control
    }
}