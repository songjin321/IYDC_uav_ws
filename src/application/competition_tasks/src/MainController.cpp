//
// Created by songjin on 18-6-4.
//

#include "competition_tasks/MainController.h"
#include "detect_track/ControlDetection.h"
MainController::MainController(std::string uav_controller_server_name, std::string object_pose_name) :
ac(uav_controller_server_name, true),is_objectPose_updated(false)
{
    ROS_INFO("Waiting for uav_controller_server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time
    ROS_INFO("uav_controller_server start!");
    //
    ros::ServiceClient detection_client = nh_.serviceClient<detect_track::ControlDetection>("control_detection_server");
    object_pose_sub = nh_.subscribe(object_pose_name, 1, &MainController::object_pose_callback, this);
}
void MainController::start_to_goal(double x, double y, double z)
{
    //　起飞到一定的高度
    goal_pose.pose.position.z = z;
    goal.goal_pose = goal_pose;
    goal.fly_vel = -1;
    goal.fly_type = "line_planner_server";
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("arrive at height of %.3f meters", z);

    // 保证姿态不变,飞到目标点
    goal_pose.pose.position.x = x;
    goal_pose.pose.position.y = y;
    goal.goal_pose = goal_pose;
    goal.fly_vel = -1;
    goal.fly_type = "line_planner_server";
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("arrive at goal point, x = %.3f, y = %.3f", x, y);
}

void MainController::sendBuzzerSignal(int seconds)
{
    //TODO::Buzzer service
}

void MainController::returnToOrigin()
{

    //　返回到原点上方
    goal_pose.pose.position.x = 0;
    goal_pose.pose.position.y = 0;
    goal.goal_pose = goal_pose;
    goal.fly_vel = -1;
    goal.fly_type = "line_planner_server";
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("return to the top of the origin");

    // 降落
    goal_pose.pose.position.z = 0;
    goal.goal_pose = goal_pose;
    goal.fly_vel = -1;
    goal.fly_type = "line_planner_server";
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("return to the origin");

    //　TODO::关闭飞机
}

void MainController::adjustUavPose()
{
    while(!is_objectPose_updated)
    {
        ros::Rate rate(30);
        ros::spinOnce();
        rate.sleep();
    }
    is_objectPose_updated = false;

    // 飞到需要调整的位置
    goal.goal_pose = object_pose;
    goal.fly_type = "line_planner_server";
    goal.fly_vel = -1;
    ac.sendGoal(goal);
    ac.waitForResult();
    ROS_INFO("return to the origin");
}
void MainController::object_pose_callback(const geometry_msgs::PoseStamped &msg)
{
    object_pose = msg;
    is_objectPose_updated = true;
}
void MainController::startObjectDetection()
{
    detect_track::ControlDetection srv;
    srv.request.ControlType = 2;
    srv.request.Start = true;
    if (detection_client.call(srv))
        ROS_INFO("detection start");
}

void MainController::stopObjectDetection()
{
    detect_track::ControlDetection srv;
    srv.request.ControlType = 2;
    srv.request.Start = false;
    if (detection_client.call(srv))
        ROS_INFO("detection stop");

}