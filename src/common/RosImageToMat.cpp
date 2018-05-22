//
// Created by songjin on 18-5-22.
//

#include "RosImageToMat.h"
#include <string>
RosImageToMat::RosImageToMat(std::string topic_name)
        : it_(nh_), topic_name_(topic_name)
{
    image_sub_ = it_.subscribe(topic_name_, 1,
                               &ImageConverter::imageCb, this);
}
void RosImageToMat::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    image = cv_ptr->image;
}
void RosImageToMat::getImage(cv::Mat &img)
{
    img = image_.clone();
}