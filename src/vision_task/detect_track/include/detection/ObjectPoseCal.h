//
// Created by songjin on 18-5-31.
//

#ifndef DETECT_TRACK_OBJECTPOSEPUB_H
#define DETECT_TRACK_OBJECTPOSEPUB_H

#include <string>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>

/**
 * 计算物体在相机坐标系的位姿,并发布
 */
class ObjectPoseCal
{
public:
    ObjectPoseCal(const std::string &camera_info_name, const std::string &publish_pose_name, 
                             double x_cam2body, double y_cam2body);
    void cameraInfoCallBack(sensor_msgs::CameraInfoConstPtr camera_info);
    void publishPose();
    void calculatePoseFromBox(const cv::Rect_<float> &box);
    void calculatePoseFromRotatedBox(const cv::RotatedRect &box);
    void calculatePoseFromPoint(const cv::Point2f &center);
private:
    ros::NodeHandle n_;
    ros::Publisher pub_pose_;
    ros::Subscriber cameraInfo_sub_;
    geometry_msgs::PoseStamped object_pose_;
    double fx,fy,cx,cy;

    // camera coordinate in body frame
    double x_cam2body_, y_cam2body_;
};
#endif //DETECT_TRACK_OBJECTPOSEPUB_H

