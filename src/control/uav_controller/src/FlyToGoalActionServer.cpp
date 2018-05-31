//
// Created by songjin on 18-5-25.
//

#include "uav_controller/FlyToGoalActionServer.h"
#include "ros_common/RosMath.h"
#include <thread>

FlyToGoalActionServer::FlyToGoalActionServer(std::string name, RosWrapperUAV *ros_uav) :
        as_(nh_, name, boost::bind(&FlyToGoalActionServer::executeCB, this, _1), false),
        action_name_(name),
        p_ros_uav_(ros_uav)
{
    as_.start();
}

void FlyToGoalActionServer::executeCB(const uav_controller::FlyToGoalGoalConstPtr &goal)
{
    nav_msgs::Path path;
    geometry_msgs::PoseStamped current_pose = p_ros_uav_->getCurrentPoseStamped();
    geometry_msgs::PoseStamped current_destination_pose;
    if (!generatePath(goal->fly_type,
                      current_pose,
                      goal->goal_pose,
                      path))
    {
        ROS_ERROR("plan path error!");
        return;
    }


    bool success = true;
    // publish info to the console for the user
    ROS_INFO("%s: Executing, the goal position x = %f, y = %f, z = %f",
             action_name_.c_str(), goal->goal_pose.pose.position.x,
             goal->goal_pose.pose.position.y,
             goal->goal_pose.pose.position.z);

    auto ite_path = path.poses.begin();
    while (ite_path!=path.poses.end()) {
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok()) {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
        }
        current_destination_pose = *ite_path;
        p_ros_uav_->fly_to_goal(current_destination_pose, -1.0);
        current_pose = p_ros_uav_->getCurrentPoseStamped();
        if (RosMath::calDistance(current_destination_pose, current_pose) < 0.01)
        {
            ite_path++;
            feedback_.distance = (float)RosMath::calDistance(current_pose, goal->goal_pose);
            // publish the feedback
            as_.publishFeedback(feedback_);
            ROS_INFO("arrive a waypoint");
        }
    }
    if (success) {
        current_pose = p_ros_uav_->getCurrentPoseStamped();
        result_.final_distance = (float)RosMath::calDistance(current_pose, goal->goal_pose);
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
}

bool FlyToGoalActionServer::generatePath(const std::string &path_planner_name,
                                         const geometry_msgs::PoseStamped &start_pose,
                                         const geometry_msgs::PoseStamped &goal_pose,
                                         nav_msgs::Path &path)
{
    ros::ServiceClient planner_client_;
    planner_client_  = nh_.serviceClient<nav_msgs::GetPlan>(path_planner_name);
    nav_msgs::GetPlan srv;
    srv.request.start = start_pose;
    srv.request.goal = goal_pose;
    srv.request.tolerance = 1.0;
    if(planner_client_.call(srv))
    {
        for(auto pose : srv.response.plan.poses)
        {
            ROS_INFO("get path success, x = %3f, y = %3f, z = %3f",
                     pose.pose.position.x,
                     pose.pose.position.y,
                     pose.pose.position.z);
        }
        return true;
    }
    else
    {
        ROS_ERROR("Failed to call service path planner");
        return false;
    }
}
