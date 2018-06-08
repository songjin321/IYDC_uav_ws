// task4 取医药包并放置医药包到遇险者附近
#include <ros/ros.h>
#include "competition_tasks/MainController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task4");
    ros::NodeHandle n;
    MainController main_controller("uav_controller_server", "object_pose", "/mavros/vision_pose/pose");

    // 起飞飞到药品包的位置
    double task4_medical_x, task4_medical_y, task4_medical_z;
    n.getParam("/task4_node/task4_medical_x", task4_medical_x);
    n.getParam("/task4_node/task4_medical_y", task4_medical_y);
    n.getParam("/task4_node/task4_medical_z", task4_medical_z);
    ROS_INFO("task4_medical_x = %.3f, task4_medical_y = %.3f, task4_medical_z = %.3f",
             task4_medical_x, task4_medical_y, task4_medical_z);
    main_controller.start_to_goal(task4_medical_x, task4_medical_y, task4_medical_z);

    // 开启目标检测,1表示medicalBag
    main_controller.startObjectDetection(1);

    // 调整无人机的位姿, 使飞机头朝向医药包的窄的一侧
    // 调整无人机的位置,位于医药包正上方.
    main_controller.adjustUavPosition(0,0);

    // 关闭目标检测,1表示medicalBag
    main_controller.stopObjectDetection(1);

    // 逐渐下降到１m高的抓取台上
    main_controller.flyFixedHeight(1);

    // 关闭飞机
    main_controller.shutDownUav();

    // 开始抓取医药包
    main_controller.grabObject();

    // similar to task2
    // 飞到遇险者上方
    double task4_person_x, task4_person_y, task4_person_z;
    n.getParam("/task4_node/task4_person_x", task4_person_x);
    n.getParam("/task4_node/task4_person_y", task4_person_y);
    n.getParam("/task4_node/task4_person_z", task4_person_z);
    ROS_INFO("task4_person_x = %.3f, task4_person_y = %.3f, task4_person_z = %.3f",
             task4_person_x, task4_person_y, task4_person_z);
    main_controller.start_to_goal(task4_person_x, task4_person_y, task4_person_z);

    // 开启目标检测,2表示colorPerson
    main_controller.startObjectDetection(2);

    // 调整无人机的位置,位于小人正上方
    main_controller.adjustUavPosition(0,0);

    // 发出提示５秒
    main_controller.sendBuzzerSignal(5);

    // 调整无人机的位置,位于药品放置区中心
    main_controller.adjustUavPosition(0.2,0);

    // 关闭目标检测,2表示colorPerson
    main_controller.stopObjectDetection(2);

    // 下降到0.2m
    main_controller.flyFixedHeight(0.2);

    // 放置医药包
    main_controller.releaseObject();

    // 升高到0.5m
    main_controller.flyFixedHeight(0.5);

    // 返回到起始点,降落到地面,关闭飞机
    main_controller.returnToOrigin();

    return 0;
}