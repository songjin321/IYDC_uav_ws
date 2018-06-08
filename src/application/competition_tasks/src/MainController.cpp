//
// Created by songjin on 18-6-4.
//

#include <opencv-3.3.1-dev/opencv2/core/types.hpp>
#include "competition_tasks/MainController.h"
#include "detect_track/ControlDetection.h"
MainController::MainController(std::string uav_controller_server_name,
                               std::string object_pose_name,
                               std::string uav_pose_name) :
ac(uav_controller_server_name, true),is_objectPose_updated(false)
{
    ROS_INFO("Waiting for uav_controller_server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("uav_controller_server start!");

    // init goal pose
    goal_pose.pose.orientation.w = 1.0;

    // detection_controller_server
    detection_client = nh_.serviceClient<detect_track::ControlDetection>("detection_controller_server");

    // object pose subscribe
    object_pose_sub = nh_.subscribe(object_pose_name, 1, &MainController::object_pose_callback, this);

    // uav pose subscribe
    uav_pose_sub = nh_.subscribe(uav_pose_name, 1, &MainController::uav_pose_callback, this);
}
void MainController::start_to_goal(double x, double y, double z)
{

    flyFixedHeight(z);
    flyInPlane(x, y);
}

void MainController::sendBuzzerSignal(int seconds)
{
    //TODO::Buzzer service
    ROS_INFO("bi bi bi bi bi bi bi bi");
}

void MainController::returnToOrigin()
{
    //　返回到原点上方
    flyInPlane(0.0, 0.0);

    // 降落
    flyFixedHeight(0);

    // 关闭飞机
    shutDownUav();
}

void MainController::adjustUavPose()
{
    while(!is_objectPose_updated)
    {
        ros::Rate rate(30);
        ros::spinOnce();
        rate.sleep();
    }
    //　仅仅只使用第一次检测到的目标物的位置
    is_objectPose_updated = false;
    std::cout << "try to adjust the pose of uav" << std::endl;
    // 飞到需要调整的位置和姿态
    double current_z =  goal_pose.pose.position.z;
    goal_pose = object_pose;
    goal_pose.pose.position.z = current_z;

    goal.goal_pose = goal_pose;
    goal.fly_type = "line_planner_server";
    goal.fly_vel = -1;
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("adjustUavPose OK");
}

void MainController::adjustUavPosition(double delta_x, double delta_y)
{
    while(!is_objectPose_updated)
    {
        ros::Rate rate(30);
        ros::spinOnce();
        rate.sleep();
    }
    //　仅仅只使用第一次检测到的目标物的位置
    // 飞到需要调整的位置,假定相机安装在正下方,相机ｘ方向和飞机ｘ方向重合,ｙ方向相反.
    is_objectPose_updated = false;
    ROS_INFO("try to adjust the position of uav, the position of object relative to uav, x = %.3f, y = %.3f",
             object_pose.pose.position.x, -object_pose.pose.position.y);
    goal_pose.pose.position.x = object_pose.pose.position.x + uav_pose.pose.position.x;
    goal_pose.pose.position.y = uav_pose.pose.position.y - object_pose.pose.position.y;

    goal.goal_pose = goal_pose;
    goal.fly_type = "position_line_planner_server";
    goal.fly_vel = -1;
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("adjustUavPosition OK");
}

void MainController::trackObject()
{
    // wait for object detection result
    while(!is_objectPose_updated)
    {
        ros::Rate rate(30);
        ros::spinOnce();
        rate.sleep();
    }

    // 当飞机接近原点时跟踪停止
    // 如果没有追踪上就往原点飞
    // 让飞机飞到原点可以被抢占
    double distance2origin = 10000000;
    while(distance2origin > 0.5) {
        // 0.2s 控制飞机运动一次
        ros::Rate rate(5);
        if(is_objectPose_updated)
        {
            cv::Point2f object_2d_position(static_cast<float>(object_pose.pose.position.x),
                                           static_cast<float>(object_pose.pose.position.y));
            /*
            std::deque<cv::Point2f> object_path;
            object_path.push_back(object_2d_position);
            object_path.pop_front();
            */

            // 飞到目标物的位置
            goal_pose.pose.position.x = uav_pose.pose.position.x + object_2d_position.x;
            goal_pose.pose.position.y = uav_pose.pose.position.y - object_2d_position.y;

            // 可以尝试控制速度加快飞机的运动, 可以被抢占,时刻改变飞行目标
            goal.goal_pose = goal_pose;
            goal.fly_type = "position_line_planner_server";
            goal.fly_vel = -1;
            ac.sendGoal(goal);
            ROS_INFO("try to fly to one object position, x = %.3f, y =%.3f",
                     goal_pose.pose.position.x,goal_pose.pose.position.y);
            // ac.waitForResult();
            is_objectPose_updated = false;
        } else
        {
            //　返回到原点上方
            goal_pose.pose.position.x = 0;
            goal_pose.pose.position.y = 0;
            goal.goal_pose = goal_pose;
            goal.fly_vel = -1;
            goal.fly_type = "position_line_planner_server";
            ac.sendGoal(goal);
            // ac.waitForResult();
            ROS_INFO("track lost, return to origin");
        }
        distance2origin = uav_pose.pose.position.x * uav_pose.pose.position.x +
                          uav_pose.pose.position.y * uav_pose.pose.position.y;
        ros::spinOnce();
        rate.sleep();
    }

}

void MainController::object_pose_callback(const geometry_msgs::PoseStamped &msg)
{
    object_pose = msg;
    is_objectPose_updated = true;
}
void MainController::uav_pose_callback(const geometry_msgs::PoseStamped &msg)
{
    uav_pose = msg;
}
void MainController::startObjectDetection(char detection_type)
{
    detect_track::ControlDetection srv;
    //# 0: car
    //# 1: medicalBag
    //# 2: colorPerson
    srv.request.ControlType = detection_type;
    srv.request.Start = true;
    if (detection_client.call(srv))
        ROS_INFO("detection start success");
    else
    {
        ROS_INFO("detection start failed");
    }
}

void MainController::stopObjectDetection(char detection_type)
{
    detect_track::ControlDetection srv;
    srv.request.ControlType = detection_type;
    srv.request.Start = false;
    if (detection_client.call(srv))
        ROS_INFO("detection stop success");
    else
    {
        ROS_INFO("detection start failed");
    }
}

void MainController::grabObject()
{
    //TODO::grabObject service
    ROS_INFO("grab object OK!");
}

void MainController::releaseObject()
{
    //TODO::releaseObject service
    ROS_INFO("release object OK!");
}

void MainController::shutDownUav()
{
    //　TODO::关闭飞机
    ROS_INFO("shut down uav!");
}

void MainController::flyFixedHeight(double z)
{
    //　起飞到一定的高度
    goal_pose.pose.position.z = z;
    goal.goal_pose = goal_pose;
    goal.fly_vel = -1;
    goal.fly_type = "position_line_planner_server";
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("arrive at height of %.3f meters", z);
}

void MainController::flyInPlane(double x, double y)
{
    // 高度和姿态不变,做平面运动
    goal_pose.pose.position.x = x;
    goal_pose.pose.position.y = y;
    goal.goal_pose = goal_pose;
    goal.fly_vel = -1;
    goal.fly_type = "position_line_planner_server";
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("arrive at goal point, x = %.3f, y = %.3f", x, y);
}


