// task2 将食物箱放置到遇险者附近
#include <ros/ros.h>
#include "competition_tasks/MainController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task1");
    ros::NodeHandle n;

    MainController main_controller("uav_controller_server", "object_pose", "/mavros/vision_pose/pose");

    // 起飞飞到目标点
    double task2_x, task2_y, task2_z;
    n.getParam("/task2_node/task2_x", task2_x);
    n.getParam("/task2_node/task2_y", task2_y);
    n.getParam("/task2_node/task2_z", task2_z);
    ROS_INFO("task2_x = %.3f, task2_y = %.3f, task2_z = %.3f",task2_x, task2_y, task2_z);
    main_controller.start_to_goal(task2_x, task2_y, task2_z);

    // 开启目标检测,2表示colorPerson
    main_controller.startObjectDetection(2);

    // 调整无人机的位置,位于小人正上方
    main_controller.adjustUavPosition(0,0);

    // 发出提示５秒
    main_controller.sendBuzzerSignal(5);

    // 调整无人机的位置,位于投递区中心
    main_controller.adjustUavPosition(0,0);

    // 关闭目标检测,2表示colorPerson
    main_controller.stopObjectDetection(2);

    // 下降到0.2m
    main_controller.flyFixedHeight(0.2);

    // 投递食物箱
    main_controller.releaseObject();

    // 升高到0.5m
    main_controller.flyFixedHeight(0.5);

    // 返回到起始点,降落到地面,关闭飞机
    main_controller.returnToOrigin();

    return 0;
}