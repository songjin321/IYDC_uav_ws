//
// Created by songjin on 18-6-4.
//
#include <opencv2/opencv.hpp>
#include "competition_tasks/MainController.h"
#include "detect_track/ControlDetection.h"
#include "manipulater_controller/ControlManipulater.h"
#include <thread>
#include <geometry_msgs/PoseStamped.h>
#include "ros_common/RosMath.h"
#include <mavros_msgs/CommandBool.h>

MainController::MainController(std::string uav_controller_server_name) :
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

    // detection_controller_server
    manipulater_client = nh_.serviceClient<manipulater_controller::ControlManipulater>("manipulater_server");

    // uav arming command
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    // ros message callback, 60HZ
    t_message_callback = std::thread(&MainController::ros_message_callback, this, 60);
    
    // control uav thread
    t_uav_control_loop = std::thread(&MainController::uav_control_loop, this, 10);

    // detection camera config
    camera_2_uav_x = 0.08;
    camera_2_uav_y = 0.05;

    // hover init radius, metric:m
    hover_radius = 0.5;
}

void MainController::ros_message_callback(int callback_rate)
{
    // object pose subscribe
    ros::Subscriber object_pose_sub = nh_.subscribe("/object_pose", 1, &MainController::object_pose_callback, this);

    // uav pose subscribe
    ros::Subscriber uav_pose_sub = nh_.subscribe("/mavros/local_position/pose", 1, &MainController::uav_pose_callback, this);

    ros::Rate loop_rate(callback_rate);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MainController::uav_control_loop(int loop_rate)
{
    ros::Rate rate(loop_rate);
    while (ros::ok())
    {
        goal.goal_pose = goal_pose;
        goal.fly_vel = -1;
        goal.fly_type = "position_line_planner_server";
        ac.sendGoal(goal);
        rate.sleep();
    }
}

void MainController::start_to_goal(double x, double y, double z)
{
    flyFixedHeight(z, 0.3);
    flyInPlane(x, y);
}
void MainController::sendBuzzerSignal(int seconds)
{
    manipulater_controller::ControlManipulater srv;
    srv.request.cmd = 4;
    if (manipulater_client.call(srv))
    {
        ROS_INFO("bi bi bi bi bi bi bi bi.....");
    } else
    {
        ROS_ERROR("bi .. failed");
    }
}

void MainController::returnToOrigin()
{
    //　返回到原点上方
    flyInPlane(0.0, 0.0);

    // 降落
    flyFixedHeight(-0.3);

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
    object_uav_dis = 1000000;
    ros::Rate rate(10);
    int stable_count=0;
    while(stable_count < 10)
    {
	if(object_uav_dis < 0.1) stable_count++;
        else stable_count=0;
        // 飞到需要调整的位置,假定相机安装在下方,相机ｘ方向和飞机ｘ方向重合,ｙ方向相反.
        if(is_objectPose_updated)
        {
            is_objectPose_updated = false;
            ROS_INFO("try to adjust the position of uav, the position of object relative to uav, x = %.3f, y = %.3f",
                     object_2_uav_x, object_2_uav_y);

            //　assum the yaw of uav is zero
            goal_pose.pose.position.x = uav_pose.pose.position.x + object_2_uav_x;
            goal_pose.pose.position.y = uav_pose.pose.position.y + object_2_uav_y;
        }
        // wait for object detection begin, let uav hover
        else{
            while(!uav_hover(uav_pose.pose.position.x, uav_pose.pose.position.y, hover_radius))
            {
                ROS_INFO("can not find object after hover radiu: %.3f, try to add search radius", hover_radius);
                hover_radius*=2;
            }

        }
        rate.sleep();
    }
    ROS_INFO("adjustUavPosition OK");
}
bool MainController::wait_task_over()
{
    ros::Rate rate(30);
    while (RosMath::calDistance(goal_pose.pose.position.x, goal_pose.pose.position.y,
                                uav_pose.pose.position.x, uav_pose.pose.position.y) > 0.1)
    {
        if (is_objectPose_updated)
        {
            // stable uav
            goal_pose.pose.position.x = uav_pose.pose.position.x;
            goal_pose.pose.position.y = uav_pose.pose.position.y;
            return false;
        }
        rate.sleep();
    }
    ROS_INFO("task over normally");
    return true;
}
bool MainController::uav_hover(double x, double y, double radiu)
{
    goal_pose.pose.position.x = x + radiu;
    goal_pose.pose.position.y = y;
    if(!wait_task_over()) return true;
    goal_pose.pose.position.x = x;
    goal_pose.pose.position.y = y + radiu;
    if(!wait_task_over()) return true;
    goal_pose.pose.position.x = x - radiu;
    goal_pose.pose.position.y = y;
    if(!wait_task_over()) return true;
    goal_pose.pose.position.x = x;
    goal_pose.pose.position.y = y - radiu;
    if(!wait_task_over()) return true;
    return false;
}
void MainController::trackObject()
{
    // 如果没有追踪上就往原点飞
    // 让飞机飞到原点即可停止
    double distance2origin = 10000000;
    ros::Rate rate(10);
    while(distance2origin > 0.5) {
        // 0.2s 控制飞机运动一次
        if(is_objectPose_updated)
        {
            // 飞到目标物的位置　可以尝试控制速度加快飞机的运动,时刻改变飞行目标
            goal_pose.pose.position.x = uav_pose.pose.position.x + object_2_uav_x;
            goal_pose.pose.position.y = uav_pose.pose.position.y + object_2_uav_y;
            is_objectPose_updated = false;
        } else
        {
            //　返回到原点上方
            goal_pose.pose.position.x = 0;
            goal_pose.pose.position.y = 0;
        }
        distance2origin = RosMath::calDistance(uav_pose.pose.position.x, uav_pose.pose.position.y, 0, 0);
        rate.sleep();
    }
}

void MainController::object_pose_callback(const geometry_msgs::PoseStamped &msg)
{
    object_pose = msg;
    object_2_uav_x = -object_pose.pose.position.x + camera_2_uav_x;
    object_2_uav_y = object_pose.pose.position.y + camera_2_uav_y;
    object_uav_dis = object_2_uav_x * object_2_uav_x + object_2_uav_y * object_2_uav_y;
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
    //# 2: BackgroundObject
    //# 3: redPerson
    //# 4: blackCircle
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
    manipulater_controller::ControlManipulater srv;
    srv.request.cmd = 2;
    if (manipulater_client.call(srv))
    {
        ROS_INFO("grab object OK!");
    } else
    {
        ROS_INFO("grab object failed!");
    }

}

void MainController::releaseObject()
{
    manipulater_controller::ControlManipulater srv;
    srv.request.cmd = 3;
    if (manipulater_client.call(srv))
    {
        ROS_INFO("release object OK!");
    } else
    {
        ROS_INFO("release object failed!");
    }

}

void MainController::shutDownUav()
{
    //　TODO::关闭飞机
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    if(arming_client.call(arm_cmd) && arm_cmd.response.success)
    {
         ROS_INFO("Vehicle locked");
         ROS_INFO("shut down uav!");
    }else
    { 
         ROS_INFO("shut down uav faled!");
    }

}

void MainController::flyFixedHeight(double z, double precision)
{
    //　起飞到一定的高度, x和y不变
    goal_pose.pose.position.z = z;
    goal_pose.pose.position.x = uav_pose.pose.position.x;
    goal_pose.pose.position.y = uav_pose.pose.position.y;

    ros::Rate rate(10);
    while (fabs(z - uav_pose.pose.position.z) > precision)
    {
        rate.sleep();
    }
    ROS_INFO("arrive at height, z of uav is %.3f meters", z);
}

void MainController::flyInPlane(double x, double y, double precision)
{
    // 高度和姿态不变,做平面运动
    goal_pose.pose.position.x = x;
    goal_pose.pose.position.y = y;
    goal_pose.pose.position.z = uav_pose.pose.position.z;

    ros::Rate rate(10);
    int stable_count = 0;
    while (stable_count < 20)
    {
        if(RosMath::calDistance(x,y, uav_pose.pose.position.x, uav_pose.pose.position.y) > precision) stable_count++;
        else stable_count=0;
        rate.sleep();
    }
    ROS_INFO("arrive at plane point x = %.3f, y = %.3f, the position of uav:x = %.3f, y = %.3f, z = %.3f", x, y, uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z);
}
