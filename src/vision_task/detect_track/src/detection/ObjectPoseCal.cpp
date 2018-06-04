//
// Created by songjin on 18-5-31

#include "detection/ObjectPoseCal.h"
#include "sensor_msgs/CameraInfo.h"

ObjectPoseCal::ObjectPoseCal(const std::string &camera_info_name, const std::string &publish_pose_name) {
    cameraInfo_sub_ = n_.subscribe(camera_info_name, 1, &ObjectPoseCal::cameraInfoCallBack, this);
    pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>(publish_pose_name, 1);
}

void ObjectPoseCal::cameraInfoCallBack(sensor_msgs::CameraInfoConstPtr p_camera_info)
{

}

void ObjectPoseCal::calculatePoseFormBox(const cv::Rect2f &box)
{

}

void ObjectPoseCal::publishPose() {
    pub_pose_.publish(object_pose_);
}
