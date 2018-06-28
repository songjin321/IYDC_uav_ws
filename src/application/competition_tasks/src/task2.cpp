// task2 将食物箱放置到遇险者附近
#include <ros/ros.h>
#include "competition_tasks/MainController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task1");
    ros::NodeHandle n;

    MainController main_controller("uav_controller_server");

    // 起飞飞到目标点
    double task2_x, task2_y, task2_z, task2_circle_x, task2_circle_y;
    n.getParam("/task2_node/task2_x", task2_x);
    n.getParam("/task2_node/task2_y", task2_y);
    n.getParam("/task2_node/task2_z", task2_z);
    n.getParam("/task2_node/task2_circle_x", task2_circle_x);
    n.getParam("/task2_node/task2_circle_y", task2_circle_y);

    ROS_INFO("task2_x = %.3f, task2_y = %.3f, task2_z = %.3f",task2_x, task2_y, task2_z);
    main_controller.start_to_goal(task2_x, task2_y, task2_z);

    // 开启目标检测,3表示redPerson
    main_controller.startObjectDetection(3);

    // 调整无人机的位置,位于小人正上方
    main_controller.adjustUavPosition(0,0);

    // 发出提示５秒
    main_controller.sendBuzzerSignal(5);

    // 关闭目标检测,3表示redPerson
    main_controller.stopObjectDetection(3);


    ROS_INFO("task2_circle_x = %.3f, task2_circle_y = %.3f",task2_circle_x, task2_circle_y);
    main_controller.flyInPlane(task2_circle_x, task2_circle_y, 0.05, 0.2);

    // 开启目标检测,4表示blackCircle
    main_controller.startObjectDetection(4);

    // 调整无人机的位置,位于投递区中心
    main_controller.adjustUavPosition(0,0);

    // 关闭目标检测,4表示blackCircle
    main_controller.stopObjectDetection(4);

    // 下降到0.3m
    main_controller.flyFixedHeight(0.6);
    main_controller.flyFixedHeight(0.2);

    // 投递食物箱
    main_controller.releaseObject();

    // 升高到task2_z
    main_controller.flyFixedHeight(0.6);
    main_controller.flyFixedHeight(task2_z);

    // 返回到起始点,降落到地面,关闭飞机
    main_controller.returnToOrigin();

    return 0;
}
