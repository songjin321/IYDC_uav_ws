//
// Created by songjin on 18-5-22.
//

#ifndef UAV_WS_ROSIMAGETOMAT_H
#define UAV_WS_ROSIMAGETOMAT_H
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
class RosImageToMat
{
public:
    RosImageToMat();
    void getImage(cv::Mat &img);
private:
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    cv::Mat image_;
    std::string topic_name_;
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
};
#endif //UAV_WS_ROSIMAGETOMAT_H