//
// Created by songjin on 18-5-25.
//

#include "uav_controller/RosWrapper_UAV.h"
#include "nav_msgs/GetPlan.h"
#include <iostream>
RosWrapper_UAV::RosWrapper_UAV(std::string vision_pose_name,
                               std::string path_planner_name,
                               UnmannedAerialVehicle* uav): p_uav_(uav)
{
    vision_pose_sub_ = n_.subscribe(vision_pose_name, 1, &RosWrapper_UAV::vision_pose_callback, this);
    planner_client_  = n_.serviceClient<nav_msgs::GetPlan>(path_planner_name);
    mavros_set_point_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
}

void RosWrapper_UAV::vision_pose_callback(const geometry_msgs::PoseStamped &vision_pose)
{
    p_uav_->setCurrentPoseStamped(vision_pose);
}

bool RosWrapper_UAV::fly_to_goal(const geometry_msgs::PoseStamped goal_pose)
{
    // you can use other planner service
    nav_msgs::GetPlan srv;
    srv.request.start = p_uav_->getCurrentPoseStamped();
    srv.request.goal = goal_pose;
    srv.request.tolerance = 1.0;
    if(planner_client_.call(srv))
    {
        // you can use other uav fly strategy
        p_uav_->set_path_to_goal(srv.response.plan);
        for(auto pose : srv.response.plan.poses)
        {
            ROS_INFO("set path success, x = %3f, y = %3f, z = %3f",
                     pose.pose.position.x,
                     pose.pose.position.y,
                     pose.pose.position.z);
        }
    }
    else
    {
        ROS_ERROR("Failed to call service path planner");
        return false;
    }
    while(!p_uav_->is_arrive_destination())
    {
        //fly to current target position
        p_uav_->fly_to_goal_by_path_onestep();

        // publish uav state to mavros
        mavros_set_point_pub_.publish(p_uav_->getCurrentDestinationPoseStamped());
    }
    return true;
}