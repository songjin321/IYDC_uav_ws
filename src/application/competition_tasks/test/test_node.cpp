// task1 飞到目标小人上方,使蜂鸣器响5秒
#include <ros/ros.h>
#include "competition_tasks/MainController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;

    MainController main_controller("uav_controller_server", "object_pose", "/mavros/local_position/pose");

    // 起飞飞到目标点
    double task_x, task_y, task_z;
    n.getParam("/test_node/task_x", task_x);
    n.getParam("/test_node/task_y", task_y);
    n.getParam("/test_node/task_z", task_z);
    ROS_INFO("task_x = %.3f, task_y = %.3f, task_z = %.3f",task_x, task_y, task_z);
    main_controller.start_to_goal(task_x, task_y, task_z);

    // 开启目标检测,2表示colorPerson
   // main_controller.startObjectDetection(2);

    // 调整无人机的位置,位于小人正上方
    //main_controller.adjustUavPosition(0,0);

    // 关闭目标检测,2表示colorPerson
    //main_controller.stopObjectDetection(2);

    // 发出提示５秒
    //main_controller.sendBuzzerSignal(5);

    // 返回到起始点,降落到地面,关闭飞机
    //main_controller.returnToOrigin();

    return 0;
}
