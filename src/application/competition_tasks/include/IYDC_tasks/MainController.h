//
// Created by songjin on 18-6-4.
//

#ifndef IYDC_TASKS_MAINCONTROLLER_H
#define IYDC_TASKS_MAINCONTROLLER_H

#include <ros/ros.h>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <uav_controller/FlyToGoalAction.h>

class MainController
{
public:
    MainController(std::string uav_controller_server_name);

    /*
     * 起飞飞到目标点
     */
    void start_to_goal(double x, double y, double z);

    /*
     * 开启目标检测
     */
    void openObjectDetection();

    /*
     * 关闭目标检测
     */
    void closeObjectDetection();

    /*
     * 返回原点
     */
    void returnToOrigin();

    /*
     * 控制蜂鸣器响
     */
    void sendBuzzerSignal(int seconds);

    /*
     * 控制无人机在目标物体的正上方.且机头方向对齐
     */
    void adjustUavPose(std::string object_pose_topic);

private:
    ros::NodeHandle nh_;
    uav_controller::FlyToGoalGoal goal;
    geometry_msgs::PoseStamped goal_pose;
    actionlib::SimpleActionClient<uav_controller::FlyToGoalAction> ac;
    ros::ServiceClient detection_client;
};


#endif //IYDC_TASKS_MAINCONTROLLER_H
