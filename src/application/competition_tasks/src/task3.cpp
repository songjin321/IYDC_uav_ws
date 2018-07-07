// task3 搜寻到３号遇险者后,跟随3号遇险者返回
// if tracking lost. let uav fly along given waypoints

#include <ros/ros.h>
#include "competition_tasks/MainController.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "task3");
    ros::NodeHandle n;
    MainController main_controller("uav_controller_server");

    // init task3 goal point and three wayPoints;
    double task3_x, task3_y, task3_z;
    n.getParam("/task3_node/task3_x", task3_x);
    n.getParam("/task3_node/task3_y", task3_y);
    n.getParam("/task3_node/task3_z", task3_z);
    ROS_INFO("task3_x = %.3f, task3_y = %.3f, task3_z = %.3f",task3_x, task3_y, task3_z);

    WayPoint way_point1, way_point2, way_point3, origin_point(0,0), start_point(task3_x, task3_y);
    n.getParam("/task3_node/wayPoint1_x", way_point1.x);
    n.getParam("/task3_node/wayPoint1_y", way_point1.y);
    n.getParam("/task3_node/wayPoint2_x", way_point2.x);
    n.getParam("/task3_node/wayPoint2_y", way_point2.y);
    n.getParam("/task3_node/wayPoint3_x", way_point3.x);
    n.getParam("/task3_node/wayPoint3_y", way_point3.y);
    std::vector<WayPoint> way_points;
    way_points.push_back(start_point);
    way_points.push_back(way_point1);
    way_points.push_back(way_point2);
    way_points.push_back(way_point3);
    way_points.push_back(origin_point);

    // 起飞飞到目标点
    main_controller.start_to_goal(task3_x, task3_y, task3_z);

    // 开启目标检测,1绿色背景上的目标物,用这个来控制飞机位于小车上方
    main_controller.startObjectDetection(1);

    // 调整无人机的位置,位于小人正上方
    main_controller.adjustUavPosition(0,0,0);

    // 关闭目标检测
    main_controller.stopObjectDetection();

    // 开启目标检测,4表示car
    main_controller.startObjectDetection(4);

    // 发出提示５秒
    main_controller.sing();

    // 使无人机追踪目标物
    main_controller.trackObject(way_points);

    // 关闭目标检测
    main_controller.stopObjectDetection();

    // 返回到起始点,降落到地面,关闭飞机
    main_controller.returnToOrigin();

    return 0;
}