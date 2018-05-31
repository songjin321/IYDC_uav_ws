//
// Created by songjin on 18-5-31.
//

#ifndef DETECT_TRACK_OBJECTPOSEPUB_H
#define DETECT_TRACK_OBJECTPOSEPUB_H

#include <string>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
class ObjectPosePub
{
public:
    ObjectPosePub(const std::string &publish_topic_name);
    void publish(const cv::Rect2f &box);
    void publish(const geometry_msgs::PoseStamped &pose);
private:
    ros::NodeHandle n_;
    ros::Publisher pub_center_point_;
    ros::Publisher pub_pose_;
    std_msgs::Float32MultiArray center_point_;
};
#endif //DETECT_TRACK_OBJECTPOSEPUB_H
