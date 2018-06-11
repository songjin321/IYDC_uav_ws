//
// Created by songjin on 18-6-10.
//
#include <ros/ros.h>
#include "manipulater_controller/ControlManipulater.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manipulater_server_test");
    ros::NodeHandle nh;

    ros::ServiceClient manipulater_client = nh.serviceClient
            <manipulater_controller::ControlManipulater>("manipulater_server");

    manipulater_controller::ControlManipulater srv;
    srv.request.cmd = 4;
    if (manipulater_client.call(srv))
    {
        ROS_INFO("bi bi bi bi bi bi bi bi.....");
    } else
    {
        ROS_ERROR("bi .. failed");
    }

    return 0;
}
