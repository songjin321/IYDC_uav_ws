// task3 搜寻到３号遇险者后,跟随3号遇险者返回
#include <ros/ros.h>
#include "competition_tasks/MainController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task3");

    MainController main_controller("uav_controller_server", "object_pose", std::__cxx11::string());

    // 起飞飞到目标点
    double task_1_x = 0.0;
    double task_1_y = 0.0;
    double task_1_z = 0.1;

    // main_controller.start_to_goal(task_1_x, task_1_y, task_1_z);

    // 开启目标检测,0表示car
    main_controller.startObjectDetection(0);

    // 调整无人机的位姿, 使飞机头朝向医药包的窄的一侧
    main_controller.adjustUavPose();

    // 关闭目标检测,0表示car
    main_controller.stopObjectDetection(0);
}