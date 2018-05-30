//
// Created by songjin on 18-5-25.
//

#ifndef UAV_CONTROLLER_FLYTOGOALACTION_H
#define UAV_CONTROLLER_FLYTOGOALACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <uav_controller/FlyToGoalAction.h>
#include "uav_controller/UnmannedAerialVehicle.h"
#include "uav_controller/RosWrapper_UAV.h"
/*
 *  provide a action server, let uav fly to a goal point.
 */
class FlyToGoalActionServer
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<uav_controller::FlyToGoalAction> as_;
    std::string action_name_;
    uav_controller::FlyToGoalFeedback feedback_;
    uav_controller::FlyToGoalResult result_;
public:
    FlyToGoalActionServer(std::string name,  RosWrapper_UAV* ros_uav);
    void executeCB(const uav_controller::FlyToGoalGoalConstPtr &goal);
private:
    RosWrapper_UAV *p_ros_uav_;
};


#endif //UAV_CONTROLLER_FLYTOGOALACTION_H
