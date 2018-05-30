//
// Created by songjin on 18-5-25.
//
#include "uav_controller/FlyToGoalActionServer.h"
#include "uav_controller/UnmannedAerialVehicle.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "uav_controller_server");
    UnmannedAerialVehicle uav;
    RosWrapper_UAV ros_uav("/vision_pose/pose", "/line_planner_server", &uav);
    FlyToGoalActionServer flyToGoalActionServer("uav_controller_server",&ros_uav);
    // you can add other action class object below
    // fly circle
    // fly follow
    ros::spin();

    return 0;
}