//
// Created by songjin on 18-5-25.
//

#include "uav_controller/FlyToGoalActionServer.h"

void FlyToGoalActionServer::executeCB(const uav_controller::FlyToGoalGoalConstPtr &goal)
{
    p_ros_uav_->fly_to_goal(goal->goal);
}