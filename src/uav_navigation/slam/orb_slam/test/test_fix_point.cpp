#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_broadcaster.h>

mavros_msgs::State current_state;
geometry_msgs::TwistStamped uav_twist;
geometry_msgs::PoseStamped set_pose;
geometry_msgs::PoseStamped local_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
	current_state = *msg;
}
void uav_vr_pose_attitude_set(const geometry_msgs::TwistStampedConstPtr& msg)
{   
    // set uav position
    set_pose.pose.position.x += msg->twist.linear.x;
    set_pose.pose.position.y += msg->twist.linear.y;
    set_pose.pose.position.z += msg->twist.linear.z;
    
    
    tf::Matrix3x3 obs_mat;
    obs_mat.setEulerYPR(msg->twist.angular.z,0,0);

    tf::Quaternion q_tf;
    obs_mat.getRotation(q_tf);
    set_pose.pose.orientation.x = q_tf.getX();
    set_pose.pose.orientation.y = q_tf.getY();
    set_pose.pose.orientation.z = q_tf.getZ();
    set_pose.pose.orientation.w = q_tf.getW();
     // ROS_INFO("set_pose position z value = %f ", set_pose.pose.position.z);
    
    /*
    // set uav angular along z axis
    uav_twist.twist.angular.z = msg->twist.angular.z;
    if (uav_twist.twist.angular.z > 0.2)
    {
        uav_twist.twist.angular.z -= 0.2;
        if (uav_twist.twist.angular.z > 0.5)
            uav_twist.twist.angular.z = 0.5;
    }
    else if (uav_twist.twist.angular.z < -0.2)
    {
        uav_twist.twist.angular.z += 0.2;
        if (uav_twist.twist.angular.z < -0.5)
            uav_twist.twist.angular.z = -0.5;
    }
    else
    {
        uav_twist.twist.angular.z = 0;
    }
    */ 
    
    // call vr motor service to set camera angle

}
void set_uav_local_pose(const geometry_msgs::PoseStampedConstPtr& msg)
{ 
    local_pose = *msg;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_fix_point");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("/mavros/state", 10, state_cb);
    ros::Subscriber uav_twist_expected_sub = nh.subscribe<geometry_msgs::TwistStamped>
            ("/uav_twist_expected", 10, uav_vr_pose_attitude_set);
    ros::Subscriber uav_local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("/mavros/local_position/pose",10, set_uav_local_pose);
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("/mavros/set_mode");
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("/mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::TwistStamped>
            ("/mavros/setpoint_attitude/cmd_vel", 10);


    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // set point 
    double set_point_x = 0;
    double set_point_y = 0;
    double set_point_z = 0;
    nh.param<double>("/test_uav/set_point_x",set_point_x,0);
    nh.param<double>("/test_uav/set_point_y",set_point_y,0);
    nh.param<double>("/test_uav/set_point_z",set_point_z,0);
    set_pose.pose = local_pose.pose;
    set_pose.pose.position.x = set_point_x;
    set_pose.pose.position.y = set_point_y;
    set_pose.pose.position.z = set_point_z;
    set_pose.pose.orientation.w = 1;

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(set_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
    /*
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.success){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
    */
        if( current_state.mode == "OFFBOARD")
            ROS_INFO("set OFFBOARD OK");
        local_pos_pub.publish(set_pose);
        // local_vel_pub.publish(uav_twist);
        //ROS_INFO("set uav position x = %.3f, y = %.3f, z = %.3f, q_w = %.3f\n",
        //        set_pose.pose.position.x,
	//	set_pose.pose.position.y,
	//	set_pose.pose.position.z,
	//	set_pose.pose.orientation.w);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
