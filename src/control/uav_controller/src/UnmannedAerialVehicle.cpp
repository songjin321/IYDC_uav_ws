//
// Created by songjin on 18-5-25.
//

#include "uav_controller/UnmannedAerialVehicle.h"
#include <ros/ros.h>
UnmannedAerialVehicle::UnmannedAerialVehicle()
{

}

void UnmannedAerialVehicle::fly_to_goal_by_path_onestep()
{
    current_destination_pose_;
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


