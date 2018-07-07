// task1 飞到目标小人上方,使蜂鸣器响5秒
#include <ros/ros.h>
#include "competition_tasks/MainController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task1");
    ros::NodeHandle n;

    MainController main_controller("uav_controller_server");

    double task1_x, task1_y, task1_z;
    n.getParam("/task1_node/task1_x", task1_x);
    n.getParam("/task1_node/task1_y", task1_y);
    n.getParam("/task1_node/task1_z", task1_z);
    ROS_INFO("task_1_x = %.3f, task_1_y = %.3f, task_1_z = %.3f",task1_x, task1_y, task1_z);

    // 初始化控制
    main_controller.init();

    // 起飞飞到目标点
    main_controller.start_to_goal(task1_x, task1_y, task1_z);

    // 开启目标检测,1绿色背景上的目标物
    main_controller.startObjectDetection(1);

    // 调整无人机的位置,位于小人正上方
    main_controller.adjustUavPosition(0,0,0);

    // 关闭目标检测,2表示colorPerson
    main_controller.stopObjectDetection();

    // 发出提示５秒
    main_controller.sing();

    // 返回到起始点,降落到地面,关闭飞机
    main_controller.returnToOrigin();

    return 0;
}