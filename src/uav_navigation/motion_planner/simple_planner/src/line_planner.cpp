#include "ros/ros.h"
#include "nav_msgs/GetPlan.h"
#include <tf/transform_datatypes.h>
#include "ros_common/RosMath.h"

bool calLinePath(nav_msgs::GetPlan::Request  &req,
         nav_msgs::GetPlan::Response &res)
{
    // let 0.1m as step length
    double step_length = 0.1;
    double dx = req.goal.pose.position.x - req.start.pose.position.x;
    double dy = req.goal.pose.position.y - req.start.pose.position.y;
    double dz = req.goal.pose.position.z - req.start.pose.position.z;
    double yaw_start = RosMath::getYawFromPoseStamp(req.start);
    // std::cout << "yaw_start = " << yaw_start << std::endl;
    double yaw_goal = RosMath::getYawFromPoseStamp(req.goal);
    // std::cout << "yaw_goal = " << yaw_goal << std::endl;
    double dyaw = yaw_goal - yaw_start;
    double path_length = sqrt(dx*dx + dy*dy + dz*dz);
    res.plan.header = req.start.header;
    geometry_msgs::PoseStamped planned_pose;
    double steps = path_length/step_length;
    for (int i = 1; i < steps; i++)
    {
        // Create this quaternion from roll/pitch/yaw (in radians)
        double yaw = yaw_start + i*dyaw/steps;
        tf::Quaternion q = tf::createQuaternionFromRPY(0, 0, yaw);

        //　规划的点的时间怎么确定
        planned_pose.header = req.start.header;
        planned_pose.pose.position.x = req.start.pose.position.x + i*dx/steps;
        planned_pose.pose.position.y = req.start.pose.position.y + i*dy/steps;
        planned_pose.pose.position.z = req.start.pose.position.z + i*dz/steps;
        planned_pose.pose.orientation.x = q.x();
        planned_pose.pose.orientation.y = q.y();
        planned_pose.pose.orientation.z = q.z();
        planned_pose.pose.orientation.w = q.w();
        res.plan.poses.push_back(planned_pose);
    }
    res.plan.poses.push_back(req.goal);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_planner");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("line_planner_server", calLinePath);
    ROS_INFO("Ready to calculate line path.");
    ros::spin();

    return 0;
}