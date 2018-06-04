//
// Created by songjin on 18-6-4.
//

#include "IYDC_tasks/MainController.h"
#include "detect_track/ControlDetection.h"
MainController::MainController(std::string uav_controller_server_name):
ac(uav_controller_server_name, true)
{
    ROS_INFO("Waiting for uav_controller_server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    //
    ros::ServiceClient detection_client = nh_.serviceClient<detect_track::ControlDetection>("control_detection_server");
}
void MainController::start_to_goal(double x, double y, double z)
{
    //　起飞到一定的高度
    goal_pose.pose.position.z = z;
    goal.goal_pose = goal_pose;
    goal.fly_type = "line_planner_server";
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("fly to a height of %.3f meters", z);

    // 保证姿态不变,飞到目标点
    goal_pose.pose.position.x = x;
    goal_pose.pose.position.y = y;
    goal.goal_pose = goal_pose;
    goal.fly_type = "line_planner_server";
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("fly to goal point, x = %.3f, y = %.3f", x, y);
}

void MainController::sendBuzzerSignal(int seconds)
{
    //TODO::Buzzer service
}

void MainController::returnToOrigin()
{
    //　返回到原点上方
    goal_pose.pose.position.x = 0;
    goal_pose.pose.position.y = 0;
    goal.goal_pose = goal_pose;
    goal.fly_type = "line_planner_server";
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("return to the top of the origin");

    // 降落
    goal_pose.pose.position.z = 0.2;
    goal.goal_pose = goal_pose;
    goal.fly_type = "line_planner_server";
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("return to the origin");

    //　TODO::关闭飞机
}

void MainController::adjustUavPose(std::string object_pose_topic)
{

}

void MainController::openObjectDetection()
{
    detect_track::ControlDetection srv;
    srv.request.ControlType = 1;
    if (detection_client.call(srv))
        ROS_INFO("detection opened");
}

void MainController::closeObjectDetection()
{
    detect_track::ControlDetection srv;
    srv.request.ControlType = 0;
    if (detection_client.call(srv))
        ROS_INFO("detection closed");
}