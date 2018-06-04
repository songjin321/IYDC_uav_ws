// task1 取医药包并放置医药包到遇险者附近
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <uav_controller/FlyToGoalAction.h>
#include "detect_track/ControlDetection.h"
#include "competition_tasks/MainController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task1");
    ros::NodeHandle n;

    MainController main_controller("uav_controller_server", "object_pose");

    // 起飞飞到目标点
    double task_1_x = 0.0;
    double task_1_y = 0.0;
    double task_1_z = 0.2;
    main_controller.start_to_goal(task_1_x, task_1_y, task_1_z);

    // 开启目标检测
    main_controller.startObjectDetection();

    // 调整无人机的位姿
    main_controller.adjustUavPose();

    // 关闭目标检测
    main_controller.stopObjectDetection();

    // 发出提示５秒
    //main_controller.sendBuzzerSignal(5);

    //  返回到起始点
    main_controller.returnToOrigin();

    return 0;
}