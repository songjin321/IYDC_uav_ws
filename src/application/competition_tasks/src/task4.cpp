// task4 取医药包并放置医药包到遇险者附近
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <uav_controller/FlyToGoalAction.h>
int main(int argc, char **argv)
{
    ros::init(argc, argv, "task4");
    actionlib::SimpleActionClient<uav_controller::FlyToGoalAction> ac("uav_controller_server", true);
    ROS_INFO("Waiting for uav_controller_server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    // 保证当前姿态不变,起飞到飞行高度

    // 保证姿态不变,飞到目标点

    // 开启目标检测

    // 使飞机头朝向医药包的窄的一侧

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