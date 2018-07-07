//
// Created by songjin on 18-6-4.
//

#ifndef IYDC_TASKS_MAINCONTROLLER_H
#define IYDC_TASKS_MAINCONTROLLER_H

#include <ros/ros.h>
#include <string>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "uav_controller/FlyToGoalAction.h"
#include <thread>
#include <vector>

struct WayPoint
{
    WayPoint(){};
    WayPoint(double x, double y)
    {
        this->x = x;
        this->y = y;
    };
    double x = 0;
    double y = 0;
    double dis2origin() const
    {
        return x*x + y*y;
    }
};
class MainController
{
public:
    MainController(std::string uav_controller_server_name);

    void init();
    /*
     * 起飞飞到目标点
     */
    void start_to_goal(double x, double y, double z);

    /*
     * 返回原点
     */
    void returnToOrigin();

    /*
     * 开启目标检测
     */
    void startObjectDetection(char detection_type);

    /*
     * 关闭目标检测
     */
    void stopObjectDetection();

    /*
     * 控制蜂鸣器响
     */
    bool sing();

    /*
     * 抓取医药包
     */
    bool catchObject();

    /*
     * 松开抓取到的物体
     */
    bool stretchObject();

    /*
     * 控制无人机的位置,距离目标物delta_x,delta_y, 目标物距离地面距离：z_to_ground
     */
    void adjustUavPosition(double delta_x, double delta_y, double z_to_ground);

    /*
     * 控制无人机跟随目标运动
     */
    void trackObject(const std::vector<WayPoint> way_points);

    /*
     * 飞到固定的高度,维持目标位置不变
     */
    void flyFixedHeight(double z, double step_length = 0.1, double precision = 0.1, double velocity = -1);

    /*
     * 让飞机在平面进行平移,维持目标高度不变
     */
    void flyInPlane(double x, double y, double precision=0.1, double step_length = 0.1);

    /*
     * let uav hover
     */
    bool uav_hover(double x, double y, double radiu);

    /*
     * 关闭飞机
     */
    void shutDownUav();

    void ros_message_callback(int callback_rate);
    
    void uav_control_loop(int loop_rate);

    void object_pose_callback(const geometry_msgs::PoseStamped &msg);

    void uav_pose_callback(const geometry_msgs::PoseStamped &msg);
private:
    ros::NodeHandle nh_;
    uav_controller::FlyToGoalGoal goal;
    // 通过修改goal_pose来控制飞机,可以实现相对运动
    // geometry_msgs::PoseStamped goal_pose;
    actionlib::SimpleActionClient<uav_controller::FlyToGoalAction> ac;
    ros::ServiceClient detection_client;
    ros::ServiceClient manipulater_client;
    ros::ServiceClient arming_client;
    geometry_msgs::PoseStamped object_pose;
    geometry_msgs::PoseStamped uav_pose;
    std::thread t_message_callback;
    std::thread t_uav_control_loop;
    bool is_objectPose_updated;

    double hover_radius;
    double object_2_uav_x;
    double object_2_uav_y;
    double object_uav_dis;

    bool wait_task_over();

    // parameters
    double x_cam2body = 0;
    double y_cam2body = 0.23;
};


#endif //IYDC_TASKS_MAINCONTROLLER_H
