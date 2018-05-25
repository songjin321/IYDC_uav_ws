//
// Created by songjin on 18-5-25.
//

#include "uav_controller/RosWrapper_UAV.h"
#include "nav_msgs/GetPlan.h"
RosWrapper_UAV::RosWrapper_UAV(std::string vision_pose_name,
                               std::string path_planner_name,
                               UnmannedAerialVehicle* uav)
{
    vision_pose_sub_ = n_.subscribe(vision_pose_name, 1, &RosWrapper_UAV::vision_pose_callback, this);
    planner_client_  = n_.serviceClient<nav_msgs::GetPlan>("line_planner_server");
    mavros_set_point_pub_ = n_.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",100);
}

void RosWrapper_UAV::vision_pose_callback(const geometry_msgs::PoseStamped &vision_pose)
{
    p_uav_->setCurrentPoseStamped(vision_pose);
}

bool RosWrapper_UAV::fly_to_goal(const geometry_msgs::PoseStamped &goal_pose)
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
    }
    else
    {
        ROS_ERROR("Failed to call service plan_line_path");
        return false;
    }
    while(!p_uav_->is_arrive_destination())
    {
        // you can use other uav fly strategy
        p_uav_->fly_to_goal_by_path_onestep();
        //fly to current target position
        mavros_set_point_pub_.publish(p_uav_->getCurrentDestinationPoseStamped());
    }
    return true;
}