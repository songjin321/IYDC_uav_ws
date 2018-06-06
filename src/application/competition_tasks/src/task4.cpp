// task4 取医药包并放置医药包到遇险者附近
#include <ros/ros.h>
#include "competition_tasks/MainController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task4");

    MainController main_controller("uav_controller_server", "object_pose");

    // 起飞飞到目标点
    double task_1_x = 0.0;
    double task_1_y = 0.0;
    double task_1_z = 0.1;
    // main_controller.start_to_goal(task_1_x, task_1_y, task_1_z);

    // 开启目标检测,1表示medicalBag
    main_controller.startObjectDetection(1);

    // 调整无人机的位姿, 使飞机头朝向医药包的窄的一侧
    main_controller.adjustUavPose();

    // 关闭目标检测,1表示medicalBag
    main_controller.stopObjectDetection(1);

    // 逐渐下降到地面

    // 开始抓取医药包

    //　起飞到飞行高度

    // 飞到遇险者上方

    //　检测遇险者,调整飞机在其正上方

    // 发出５秒的营救信号

    // 下降到一定的高度放下药品

    // 返回到起始点,并降落
    ROS_INFO("Action uav_controller_server started, try to fly!!!.");
}