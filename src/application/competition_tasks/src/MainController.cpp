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
        ac(uav_controller_server_name, true), is_objectPose_updated(false) {
    ROS_INFO("Waiting for uav_controller_server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("uav_controller_server start!");

    // init goal pose and goal type
    goal.goal_pose.pose.orientation.w = 1.0;
    goal.fly_vel = -1;
    goal.fly_type = "position_line_planner_server";
    goal.step_length = 0.1;

    // detection_controller_server
    detection_client = nh_.serviceClient<detect_track::ControlDetection>("detection_controller_server");

    // detection_controller_server
    manipulater_client = nh_.serviceClient<manipulater_controller::ControlManipulater>("manipulater_server");

    // uav arming command
    arming_client = nh_.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");

    // ros message callback, 60HZ
    t_message_callback = std::thread(&MainController::ros_message_callback, this, 60);

    // control uav thread
    t_uav_control_loop = std::thread(&MainController::uav_control_loop, this, 5);

    // hover init radius, metric:m
    hover_radius = 0.25;
}

void MainController::ros_message_callback(int callback_rate) {
    // object pose subscribe
    ros::Subscriber object_pose_sub = nh_.subscribe("/object_pose", 1, &MainController::object_pose_callback, this);

    // uav pose subscribe
    ros::Subscriber uav_pose_sub = nh_.subscribe("/mavros/local_position/pose", 1, &MainController::uav_pose_callback,
                                                 this);

    ros::Rate loop_rate(callback_rate);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void MainController::uav_control_loop(int loop_rate) {
    ros::Rate rate(loop_rate);
    while (ros::ok()) {
        // std::cout << "step length = " << goal.step_length << std::endl;
        // std::cout << "z =  " << goal_pose.pose.position.z << std::endl;
        ac.sendGoal(goal);
        rate.sleep();
    }
}

void MainController::init() {
    goal.goal_pose.pose.position.x = uav_pose.pose.position.x;
    goal.goal_pose.pose.position.y = uav_pose.pose.position.y;
}

void MainController::start_to_goal(double x, double y, double z) {
    // 起飞
    flyFixedHeight(z, 0.3, 0.1);

    // 飞到目标点
    flyInPlane(x, y, 0.3, 0.3);
}

void MainController::returnToOrigin() {
    //　返回到原点上方
    flyInPlane(0.0, 0.0, 0.3, 0.3);

    // 降落
    flyFixedHeight(-0.05);
}

void MainController::adjustUavPosition(double delta_x, double delta_y, double z_to_ground) {
    double object_uav_dis = 1000000;
    double object_2_uav_x_real;
    double object_2_uav_y_real;
    ros::Rate rate(10);
    int stable_count = 0;
    while (stable_count < 20) {
        if (object_uav_dis < 0.07) {
            // ROS_INFO("object_uav_dis = %.3f", object_uav_dis);
            ROS_INFO("stable count = %d", stable_count);
            stable_count++;
        } else stable_count = 0;
        // 飞到需要调整的位置,假定相机安装在下方,相机ｘ方向和飞机ｘ方向重合,ｙ方向相反.
        if (is_objectPose_updated) {
            is_objectPose_updated = false;
            object_2_uav_x_real = object_2_uav_x * (uav_pose.pose.position.z - z_to_ground) + x_cam2body;
            object_2_uav_y_real = object_2_uav_y * (uav_pose.pose.position.z - z_to_ground) + y_cam2body;
            object_uav_dis = sqrt(pow(object_2_uav_x_real, 2) + pow(object_2_uav_x_real, 2));
            ROS_INFO("try to adjust uav position, the position of object relative to uav, "
                     "x = %.3f, y = %.3f", object_2_uav_x_real, object_2_uav_y_real);
            //　assum the yaw of uav is not zero
            double yaw = RosMath::getYawFromPoseStamp(uav_pose);
            goal.goal_pose.pose.position.x = uav_pose.pose.position.x +
                                             object_2_uav_x_real * cos(yaw) - object_2_uav_x_real * sin(yaw);
            goal.goal_pose.pose.position.y = uav_pose.pose.position.y +
                                             object_2_uav_x_real * sin(yaw) + object_2_uav_x_real * cos(yaw);;
        }
            // wait for object detection begin, let uav hover
        else {
            while (!uav_hover(uav_pose.pose.position.x, uav_pose.pose.position.y, hover_radius)) {
                ROS_INFO("can not find object after hover radiu: %.3f, try to add search radius", hover_radius);
                // lab
                return;
                hover_radius *= 2;
            }

        }
        rate.sleep();
    }
    ROS_INFO("adjustUavPosition OK");
}

bool MainController::wait_task_over() {
    ros::Rate rate(30);
    while (RosMath::calDistance(goal.goal_pose.pose.position.x, uav_pose.pose.position.x,
                                goal.goal_pose.pose.position.y, uav_pose.pose.position.y) > 0.05) {
        if (is_objectPose_updated) {
            // stable uav
            goal.goal_pose.pose.position.x = uav_pose.pose.position.x;
            goal.goal_pose.pose.position.y = uav_pose.pose.position.y;
            return false;
        }
        rate.sleep();
    }
    ROS_INFO("task over normally");
    return true;
}

bool MainController::uav_hover(double x, double y, double radiu) {
    goal.goal_pose.pose.position.x = x + radiu;
    goal.goal_pose.pose.position.y = y;
    if (!wait_task_over()) return true;
    goal.goal_pose.pose.position.x = x;
    goal.goal_pose.pose.position.y = y + radiu;
    if (!wait_task_over()) return true;
    goal.goal_pose.pose.position.x = x - radiu;
    goal.goal_pose.pose.position.y = y;
    if (!wait_task_over()) return true;
    goal.goal_pose.pose.position.x = x;
    goal.goal_pose.pose.position.y = y - radiu;
    if (!wait_task_over()) return true;
    return false;
}

void MainController::trackObject(const std::vector <WayPoint> way_points) {
    // 如果没有追踪上就往原点飞
    // 让飞机飞到原点即可停止
    double uav2origin = 10000000;
    double object_2_uav_x_real;
    double object_2_uav_y_real;
    ros::Rate rate(20);
    while (uav2origin > 0.3) {
        // 0.2s 控制飞机运动一次
        if (is_objectPose_updated) {
            // 飞到目标物的位置　可以尝试控制速度加快飞机的运动,时刻改变飞行目标
            object_2_uav_x_real = object_2_uav_x * uav_pose.pose.position.z + x_cam2body;
            object_2_uav_y_real = object_2_uav_y * uav_pose.pose.position.z + y_cam2body;
            ROS_INFO("try to adjust uav position, the position of object relative to uav, "
                     "x = %.3f, y = %.3f", object_2_uav_x_real, object_2_uav_y_real);
            //　assum the yaw of uav is not zero
            double yaw = RosMath::getYawFromPoseStamp(uav_pose);
            goal.goal_pose.pose.position.x = uav_pose.pose.position.x +
                                             object_2_uav_x_real * cos(yaw) - object_2_uav_x_real * sin(yaw);
            goal.goal_pose.pose.position.y = uav_pose.pose.position.y +
                                             object_2_uav_x_real * sin(yaw) + object_2_uav_x_real * cos(yaw);;
            // fly to target point with 1m/s velocity
            goal.fly_vel = 1;
            goal.step_length = 0.5;
            is_objectPose_updated = false;
        } else {
            //　fly along way points, assume the all waypoints organized from far to close.
            // step1: find two wayPoints closest to uav;
            int index = 0;
            double dis_min = 100000;
            for (int i = 0; i < way_points.size(); i++) {
                double dis_x = way_points[i].x - uav_pose.pose.position.x;
                double dis_y = way_points[i].y - uav_pose.pose.position.y;
                double distance = sqrt(dis_x * dis_x + dis_y * dis_y);
                if (distance < dis_min) {
                    dis_min = distance;
                    index = i;
                }
            }
            ROS_INFO("closest wayPoint index = %d, dis_min = %.3f", index, dis_min);
            if (index == 0) {
                goal.goal_pose.pose.position.x = way_points[1].x;
                goal.goal_pose.pose.position.y = way_points[1].y;
            } else if (index == way_points.size()) {
                goal.goal_pose.pose.position.x = (way_points.end() - 1)->x;
                goal.goal_pose.pose.position.y = (way_points.end() - 1)->y;
            } else if (dis_min < 0.2) {
                goal.goal_pose.pose.position.x = way_points[index + 1].x;
                goal.goal_pose.pose.position.y = way_points[index + 1].y;
            } else if (pow(way_points[index - 1].x - uav_pose.pose.position.x, 2)
                       + pow(way_points[index - 1].y - uav_pose.pose.position.y, 2) >
                       pow(way_points[index + 1].x - uav_pose.pose.position.x, 2)
                       + pow(way_points[index + 1].y - uav_pose.pose.position.y, 2)) {
                goal.goal_pose.pose.position.x = way_points[index + 1].x;
                goal.goal_pose.pose.position.y = way_points[index + 1].y;
            } else {
                goal.goal_pose.pose.position.x = way_points[index].x;
                goal.goal_pose.pose.position.y = way_points[index].y;
            }
            goal.fly_vel = 1;
            goal.step_length = 0.5;
            ROS_INFO("track lost, fly along wayPoints, waypoint: x = %.3f, y = %.3f, velocity of uav: v = %.3f",
                     goal.goal_pose.pose.position.x, goal.goal_pose.pose.position.y, goal.fly_vel);
        }
        uav2origin = RosMath::calDistance(uav_pose.pose.position.x, 0, uav_pose.pose.position.y, 0);
        rate.sleep();
    }

    // reset fly_vel
    goal.fly_vel = -1;
}

void MainController::object_pose_callback(const geometry_msgs::PoseStamped &msg) {
    object_pose = msg;
    object_2_uav_x = object_pose.pose.position.x;
    object_2_uav_y = object_pose.pose.position.y;
    is_objectPose_updated = true;
}

void MainController::uav_pose_callback(const geometry_msgs::PoseStamped &msg) {
    uav_pose = msg;
}

void MainController::startObjectDetection(char detection_type) {
    detect_track::ControlDetection srv;
    srv.request.ControlType = detection_type;
    if (detection_client.call(srv))
        ROS_INFO("detection start success");
    else {
        ROS_INFO("detection start failed");
    }
}

void MainController::stopObjectDetection() {
    detect_track::ControlDetection srv;
    srv.request.ControlType = 0;
    if (detection_client.call(srv))
        ROS_INFO("detection stop success");
    else {
        ROS_INFO("detection start failed");
    }
}

bool MainController::catchObject() {
    manipulater_controller::ControlManipulater srv;
    srv.request.cmd = 3;
    if (manipulater_client.call(srv)) {
        ROS_INFO("catch object OK!");
        return true;
    } else {
        ROS_INFO("catch object failed!");
        return false;
    }
}

bool MainController::stretchObject() {
    manipulater_controller::ControlManipulater srv;
    srv.request.cmd = 4;
    if (manipulater_client.call(srv)) {
        ROS_INFO("stretch object OK!");
        return true;
    } else {
        ROS_INFO("stretch object failed!");
    }
    return false;
}

bool MainController::sing() {
    manipulater_controller::ControlManipulater srv;
    srv.request.cmd = 5;
    if (manipulater_client.call(srv)) {
        ROS_INFO("bi .. Ok");
        return true;
    } else {
        ROS_ERROR("bi .. failed");
    }
    return false;
}

void MainController::shutDownUav() {
    //　TODO::关闭飞机
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = false;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle locked");
        ROS_INFO("shut down uav!");
    } else {
        ROS_INFO("shut down uav faled!");
    }

}

void MainController::flyFixedHeight(double z, double step_length, double precision, double velocity) {
    //　起飞到一定的高度, x和y不变
    goal.goal_pose.pose.position.z = z;
    goal.fly_vel = velocity;
    goal.step_length = step_length;
    ROS_INFO("try to arrive at height %.3f meters, step_length = %.3f, "
             "precision = %.3f, velocity = %.3f", z, step_length, precision, velocity);

    ros::Rate rate(10);
    int stable_count = 0;
    while (stable_count < 10) {
        if (fabs(z - uav_pose.pose.position.z) < precision) stable_count++;
        else stable_count = 0;
        rate.sleep();
        // std::cout << "stable count = " << stable_count << std::endl;
    }
    ROS_INFO("arrive at goal height, the actual height of uav is %.3f meters", uav_pose.pose.position.z);
}

void MainController::flyInPlane(double x, double y, double step_length, double precision) {
    // 高度和姿态不变,做平面运动
    goal.goal_pose.pose.position.x = x;
    goal.goal_pose.pose.position.y = y;
    goal.step_length = step_length;

    ROS_INFO("try to arrive at plane point x = %.3f, y = %.3f, z = %.3f, "
             "precision = %.3f, step_length = %.3f", x, y, goal.goal_pose.pose.position.z,
             precision, step_length);

    ros::Rate rate(10);
    int stable_count = 0;
    while (stable_count < 10) {
        if (RosMath::calDistance(x, uav_pose.pose.position.x, y, uav_pose.pose.position.y) < precision) stable_count++;
        else stable_count = 0;
        rate.sleep();
        // std::cout << "stable count = " << stable_count << std::endl;
    }
    ROS_INFO("arrive at goal plane point, the position of uav:x = %.3f, y = %.3f, z = %.3f",
             uav_pose.pose.position.x, uav_pose.pose.position.y, uav_pose.pose.position.z);
}
