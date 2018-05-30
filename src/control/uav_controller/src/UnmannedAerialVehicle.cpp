//
// Created by songjin on 18-5-25.
//

#include "uav_controller/UnmannedAerialVehicle.h"
#include <ros/ros.h>
#include "ros_common/RosMath.h"
UnmannedAerialVehicle::UnmannedAerialVehicle()
{
    uav_state_ = UAV_State::begin_fly;
}

void UnmannedAerialVehicle::fly_to_goal_by_path_onestep()
{
    if (path_to_goal_.poses.empty())
    {
        uav_state_ = UAV_State::arrive_destination;
        return;
    }
    current_destination_pose_ = path_to_goal_.poses[0];
    if (RosMath::calDistance(current_destination_pose_, current_pose_) < 0.01)
    {
        path_to_goal_.poses.erase(path_to_goal_.poses.begin());
        ROS_INFO("arrive a waypoint");
    }
}

geometry_msgs::PoseStamped UnmannedAerialVehicle::getCurrentPoseStamped()
{
    return current_pose_;
}

void UnmannedAerialVehicle::setCurrentPoseStamped(const geometry_msgs::PoseStamped &vision_pose)
{
    current_pose_ = vision_pose;
}

geometry_msgs::PoseStamped UnmannedAerialVehicle::getCurrentDestinationPoseStamped()
{
    return current_destination_pose_;
}

void UnmannedAerialVehicle::set_path_to_goal(const nav_msgs::Path &path_to_goal)
{
    path_to_goal_ = path_to_goal;
}


