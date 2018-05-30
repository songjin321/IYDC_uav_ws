//
// Created by songjin on 18-5-25.
//

#include "uav_controller/FlyToGoalActionServer.h"
#include "ros_common/RosMath.h"
#include <thread>

FlyToGoalActionServer::FlyToGoalActionServer(std::string name, RosWrapper_UAV *ros_uav) :
        as_(nh_, name, boost::bind(&FlyToGoalActionServer::executeCB, this, _1), false),
        action_name_(name),
        p_ros_uav_(ros_uav)
{
    as_.start();
}

void FlyToGoalActionServer::executeCB(const uav_controller::FlyToGoalGoalConstPtr &goal) {
    // helper variables
    ros::Rate r(1);
    bool success = true;
    // publish info to the console for the user
    ROS_INFO("%s: Executing, the goal position x = %f, y = %f, z = %f",
             action_name_.c_str(), goal->goal_pose.pose.position.x,
             goal->goal_pose.pose.position.y,
             goal->goal_pose.pose.position.z);

    // 在另一个线程里运行fly_to_goal
    std::thread th(&RosWrapper_UAV::fly_to_goal, p_ros_uav_, goal->goal_pose);
    p_ros_uav_->p_uav_->set_state_begin_fly();
    geometry_msgs::PoseStamped current_pose;
    while (!p_ros_uav_->p_uav_->is_arrive_destination()) {
        // check that preempt has not been requested by the client
        if (as_.isPreemptRequested() || !ros::ok()) {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
        }
        current_pose = p_ros_uav_->p_uav_->getCurrentPoseStamped();
        feedback_.distance = RosMath::calDistance(current_pose, goal->goal_pose);

        // publish the feedback
        as_.publishFeedback(feedback_);

        r.sleep();
    }
    if (success) {
        current_pose = p_ros_uav_->p_uav_->getCurrentPoseStamped();
        result_.final_distance = RosMath::calDistance(current_pose, goal->goal_pose);
        ROS_INFO("%s: Succeeded", action_name_.c_str());
        // set the action state to succeeded
        as_.setSucceeded(result_);
    }
    th.join();

}