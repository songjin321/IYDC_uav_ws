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

class FlyToGoalActionServer
{
protected:
    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<uav_controller::FlyToGoalAction> as_;
    std::string action_name_;
    uav_controller::FlyToGoalActionFeedback feedback_;
    uav_controller::FlyToGoalActionResult result_;
public:
    FlyToGoalActionServer(std::string name,  RosWrapper_UAV* ros_uav):
            as_(nh_, name, boost::bind(&FlyToGoalActionServer::executeCB, this, _1), false),
            action_name_(name),
            p_ros_uav_(ros_uav)
    {
        as_.start();
    }
    void executeCB(const uav_controller::FlyToGoalGoalConstPtr &goal);
private:
    RosWrapper_UAV *p_ros_uav_;
};


#endif //UAV_CONTROLLER_FLYTOGOALACTION_H
