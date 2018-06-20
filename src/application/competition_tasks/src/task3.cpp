// task3 搜寻到３号遇险者后,跟随3号遇险者返回
#include <ros/ros.h>
#include "competition_tasks/MainController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task3");
    ros::NodeHandle n;
    MainController main_controller("uav_controller_server");

    // 起飞飞到目标点
    double task3_x, task3_y, task3_z;
    n.getParam("/task3_node/task3_x", task3_x);
    n.getParam("/task3_node/task3_y", task3_y);
    n.getParam("/task3_node/task3_z", task3_z);
    ROS_INFO("task3_x = %.3f, task3_y = %.3f, task3_z = %.3f",task3_x, task3_y, task3_z);
    main_controller.start_to_goal(task3_x, task3_y, task3_z);

    // 开启目标检测,2表示BackgroundObject,用这个来控制飞机位于小车上方
    main_controller.startObjectDetection(2);

    // 调整无人机的位置,位于小人正上方
    main_controller.adjustUavPosition(0,0);

    // 关闭目标检测,2表示BackgroundObject
    main_controller.stopObjectDetection(2);

    // 发出提示５秒
    main_controller.sendBuzzerSignal(5);

    // 开启目标检测,0表示car
    main_controller.startObjectDetection(0);

    // 使无人机追踪目标物
    main_controller.trackObject();

    // 关闭目标检测,0表示car
    main_controller.stopObjectDetection(0);

    // 返回到起始点,降落到地面,关闭飞机
    main_controller.returnToOrigin();

    return 0;
}