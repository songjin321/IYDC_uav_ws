// task4 取医药包并放置医药包到遇险者附近
#include <ros/ros.h>
#include "competition_tasks/MainController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task4");
    ros::NodeHandle n;
    MainController main_controller("uav_controller_server");

    double task4_medical_x, task4_medical_y, task4_medical_z, task4_z;
    n.getParam("/task4_node/task4_medical_x", task4_medical_x);
    n.getParam("/task4_node/task4_medical_y", task4_medical_y);
    n.getParam("/task4_node/task4_medical_z", task4_medical_z);
    n.getParam("/task4_node/task4_z", task4_z);
    ROS_INFO("task4_medical_x = %.3f, task4_medical_y = %.3f, task4_z = %.3f, "
             "task4_medical_z = %.3f",
             task4_medical_x, task4_medical_y, task4_z, task4_medical_z);

    // 起飞飞到药品包的位置
    main_controller.start_to_goal(task4_medical_x, task4_medical_y, task4_z);

    // 开启目标检测,5表示medicalBag
    main_controller.startObjectDetection(5);

    // 调整无人机的位置,位于医药包正上方.
    main_controller.adjustUavPosition(0,0,task4_medical_z);

    // 关闭目标检测
    main_controller.stopObjectDetection();

    // 逐渐下降到抓取台上
    main_controller.flyFixedHeight(task4_medical_z,0.1,0.05);

    // 关闭飞机
    // main_controller.shutDownUav();

    // 开始抓取医药包,最多尝试3次
    for (int i = 0; i < 3; i++)
    {
        if(main_controller.catchObject())
        {
            break;
        } else{
            // 松开爪子
            main_controller.stretchObject();

            // 飞高
            main_controller.flyFixedHeight(task4_z, 0.3, 0.1);

            // 开启目标检测,5表示medicalBag
            main_controller.startObjectDetection(5);

            // 调整无人机的位置,位于医药包正上方.
            main_controller.adjustUavPosition(0,0,task4_medical_z);

            // 关闭目标检测
            main_controller.stopObjectDetection();

            // 逐渐下降到抓取台上
            main_controller.flyFixedHeight(task4_medical_z,0.1,0.05);
        }
    }

    // similar to task2
    // 飞到遇险者上方
    double task4_person_x, task4_person_y;
    n.getParam("/task4_node/task4_person_x", task4_person_x);
    n.getParam("/task4_node/task4_person_y", task4_person_y);
    ROS_INFO("task4_person_x = %.3f, task4_person_y = %.3f", task4_person_x, task4_person_y);

    main_controller.start_to_goal(task4_person_x, task4_person_y, task4_z);

    // 开启目标检测,6表示yellowPerson
    main_controller.startObjectDetection(6);

    // 调整无人机的位置,位于小人正上方
    main_controller.adjustUavPosition(0,0,0);

    // 发出提示５秒
    main_controller.sing();

    // 关闭目标检测
    main_controller.stopObjectDetection();

    double task4_place_x, task4_place_y, task4_place_z;
    n.getParam("/task4_node/task4_place_x", task4_place_x);
    n.getParam("/task4_node/task4_place_y", task4_place_y);
    n.getParam("/task4_node/task4_place_z", task4_place_z);
    ROS_INFO("task4_place_x = %.3f, task4_place_y = %.3f, task4_place_z = %.3f",
             task4_place_x, task4_place_y, task4_place_z);

    // 调整无人机的位置,位于药品放置区中心
    main_controller.flyInPlane(task4_place_x, task4_place_y);

    // 开启目标检测,2表示redPerson
    main_controller.startObjectDetection(2);

    // 调整无人机的位置,位于放置台正上方
    main_controller.adjustUavPosition(0,0,task4_place_z);

    // 下降到放置台上
    main_controller.flyFixedHeight(task4_place_z);

    // 放置医药包
    main_controller.stretchObject();

    // 升高到固定高度
    main_controller.flyFixedHeight(task4_z, 0.3, 0.1);

    // 返回到起始点,降落到地面,关闭飞机
    main_controller.returnToOrigin();

    return 0;
}