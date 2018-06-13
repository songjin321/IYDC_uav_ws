#include "ros/ros.h"
#include <cmath>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>

using namespace std;

ros::Publisher body_puber;

// camera relative to body coordinate
double x = 0;
double y = 0;
double z = 0;

// from body rotate to camera coordinate frame, z->yaw, y->pitch, x->roll
double yaw = -M_PI_2;
double pitch = 0;
double roll = M_PI_2;


void PoseStamped_Callback(const geometry_msgs::PoseStampedConstPtr &msg)
{
    tf::Transform T_M_C;
    tf::poseMsgToTF(msg->pose, T_M_C);

    tf::Quaternion q;
    q.setEulerZYX(yaw, pitch, roll);
    // std::cout << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
	tf::Transform T_B_C = tf::Transform(q, tf::Vector3(x, y, z));

	tf::Transform T_W_B = T_B_C * T_M_C * T_B_C.inverse();

	geometry_msgs::PoseStamped current_pose;
	current_pose.header.stamp = msg->header.stamp;
	current_pose.header.frame_id = msg->header.frame_id;
	tf::poseTFToMsg(T_W_B, current_pose.pose);

	body_puber.publish(current_pose);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "map_world_convert");
  	ros::NodeHandle n;

	std::string vision_pose_map_name;
	std::string vision_pose_world_name;
    	nh.param<std::string>("vision_pose_map_name",vision_pose_map_name,"/vision_pose_map/pose");
	nh.param<std::string>("vision_pose_world_name",vision_pose_map_name,"/vision_pose_world/pose");

  	ros::Subscriber pose_feedback=n.subscribe(vision_pose_map_name, 1, PoseStamped_Callback);
  	body_puber = n.advertise<geometry_msgs::PoseStamped>(vision_pose_world_name, 1);
  	
  	ros::spin();

	return 0;

}
