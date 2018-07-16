// task1 飞到目标小人上方,使蜂鸣器响5秒
#include <ros/ros.h>
#include "competition_tasks/MainController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_node");
    ros::NodeHandle n;

    MainController main_controller("uav_controller_server");

    // 起飞飞到目标点
    double task_x, task_y, task_z, task_height;
    int object_id;
    n.getParam("/test_node/task_x", task_x);
    n.getParam("/test_node/task_y", task_y);
    n.getParam("/test_node/task_z", task_z);
    n.getParam("/test_node/task_height", task_height);
    n.getParam("/test_node/object_id", object_id);
    ROS_INFO("task_x = %.3f, task_y = %.3f, task_z = %.3f, task_height = %.3f, object_id = %d",
              task_x, task_y, task_z, task_height, object_id);
    std::vector<WayPoint> way_points;

    // 初始化控制
    main_controller.init();

    // 起飞飞到目标点
    main_controller.start_to_goal(task_x, task_y, task_z);

    // 开启目标检测,1绿色背景上的目标物
    main_controller.startObjectDetection(object_id);

    // 调整无人机的位置,位于小人正上方
    main_controller.adjustUavPosition(0,0,task_height, 0.07);

    // 关闭目标检测,2表示colorPerson
    main_controller.stopObjectDetection();

    // 发出提示５秒
    main_controller.sing();

    // 下降到放置台上
    main_controller.flyFixedHeight(task_height+0.1,0.2,0.2);

    // 放置医药包
    main_controller.stretchObject();

    // 升高到固定高度
    main_controller.flyFixedHeight(task_z, 0.2, 0.1);

    // 返回到起始点,降落到地面,关闭飞机
    main_controller.returnToOrigin();

    return 0;
}
