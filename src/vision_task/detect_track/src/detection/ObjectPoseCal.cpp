//
// Created by songjin on 18-5-31

#include "detection/ObjectPoseCal.h"
#include "sensor_msgs/CameraInfo.h"

ObjectPoseCal::ObjectPoseCal(const std::string &camera_info_name, const std::string &publish_pose_name)
{
    cameraInfo_sub_ = n_.subscribe(camera_info_name, 1, &ObjectPoseCal::cameraInfoCallBack, this);
    pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>(publish_pose_name, 1);
}

void ObjectPoseCal::cameraInfoCallBack(sensor_msgs::CameraInfoConstPtr p_camera_info)
{
    fx = p_camera_info->K[0];
    fy = p_camera_info->K[4];
    cx = p_camera_info->K[2];
    cy = p_camera_info->K[5];
}

void ObjectPoseCal::calculatePoseFromBox(const cv::Rect2f &box)
{
    object_pose_.pose.position.x = -(box.x+box.width/2-cx)/fx;
    object_pose_.pose.position.y = (box.y+box.height/2-cy)/fy;
}

void ObjectPoseCal::calculatePoseFromPoint(const cv::Point2f &center)
{
    object_pose_.pose.position.x = -(center.x-cx)/fx;
    object_pose_.pose.position.y = (center.y-cy)/fy;
}

void ObjectPoseCal::calculatePoseFromRotatedBox(const cv::RotatedRect &box)
{
    object_pose_.pose.position.x = -(box.center.x-cx)/fx;
    object_pose_.pose.position.y = (box.center.y-cy)/fy;
    // std::cout << "the yaw angle of the box  = " << box.angle << std::endl;
    object_pose_.pose.orientation.w = cos(-box.angle/180*M_PI/2);
    object_pose_.pose.orientation.z = sin(-box.angle/180*M_PI/2);
    object_pose_.pose.orientation.y = 0;
    object_pose_.pose.orientation.x = 0;
}
// 0->background 1->object
void ObjectPoseCal::publishPose(int object_type)
{
    object_pose_.pose.position.z == object_type;
    pub_pose_.publish(object_pose_);
}
