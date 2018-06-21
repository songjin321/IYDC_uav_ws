// task1 飞到目标小人上方,使蜂鸣器响5秒
#include <ros/ros.h>
#include "competition_tasks/MainController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;

    MainController main_controller("uav_controller_server");

    // 起飞飞到目标点
    double task_x, task_y, task_z;
    int object_id;
    n.getParam("/test_node/task_x", task_x);
    n.getParam("/test_node/task_y", task_y);
    n.getParam("/test_node/task_z", task_z);
    n.getParam("/test_node/object_id", object_id);
    ROS_INFO("task_x = %.3f, task_y = %.3f, task_z = %.3f, object_id = %d",
              task_x, task_y, task_z, object_id);
    main_controller.start_to_goal(task_x, task_y, task_z);
    // 开启目标检测,2表示colorPerson
    main_controller.startObjectDetection(object_id);

    // 调整无人机的位置,位于小人正上方
    main_controller.adjustUavPosition(0,0);

    // 关闭目标检测,2表示colorPerson
    main_controller.stopObjectDetection(object_id);

    // 发出提示５秒
    ROS_INFO("bi bi bi bi bi");
    ros::Duration(5).sleep();

    // 返回到起始点,降落到地面,关闭飞机
    main_controller.returnToOrigin();

    return 0;
}
