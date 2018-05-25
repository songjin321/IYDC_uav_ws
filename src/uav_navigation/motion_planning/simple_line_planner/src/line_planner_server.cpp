#include "ros/ros.h"
#include "nav_msgs/GetPlan.h"

bool calLinePath(nav_msgs::GetPlan::Request  &req,
         nav_msgs::GetPlan::Response &res)
{
    // let 0.1m as step length
    double step_length = 0.1;
    double dx = req.goal.pose.position.x - req.start.pose.position.x;
    double dy = req.goal.pose.position.y - req.start.pose.position.y;
    double dz = req.goal.pose.position.z - req.start.pose.position.z;
    double path_length = sqrt(dx*dx + dy*dy + dz*dz);
    res.plan.header = req.start.header;
    geometry_msgs::PoseStamped planned_pose;
    double steps = path_length/step_length;
    for (int i = 0; i < steps; i++)
    {
        //　规划的点的时间怎么确定
        planned_pose.header = req.start.header;
        planned_pose.pose.position.x = req.start.pose.position.x + i*dx/steps;
        planned_pose.pose.position.y = req.start.pose.position.y + i*dy/steps;
        planned_pose.pose.position.z = req.start.pose.position.z + i*dz/steps;
        planned_pose.pose.orientation = req.goal.pose.orientation;
        res.plan.poses.push_back(planned_pose);
    }
    res.plan.poses.push_back(req.goal);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "line_planner_server");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("get_line_planner", calLinePath);
    ROS_INFO("Ready to calculate line path.");
    ros::spin();

    return 0;
}