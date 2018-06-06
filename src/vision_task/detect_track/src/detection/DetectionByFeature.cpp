//
// Created by songjin on 18-5-22.
//

// OpenCV stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for homography
#include "detection/DetectionByFeature.h"
#include <string>
#include <iostream>

DetectionByFeature::DetectionByFeature()
{
    H = cv::Mat::zeros(3,3,CV_32F);
}
bool DetectionByFeature::detect(cv::Mat &sceneImg, cv::Rect2d &roi)
{
    if (scene_corners.empty())
        return false;
    roi = cv::minAreaRect(scene_corners).boundingRect2f();
    return true;
}
bool DetectionByFeature::detect(cv::Mat &sceneImg, cv::RotatedRect &roi)
{
    if (scene_corners.empty())
        return false;
    roi = cv::minAreaRect(scene_corners);
    scene_corners.clear();
    //std::cout <<"roi size = " << roi.size << std::endl;
    //std::cout <<"roi center = " << roi.center << std::endl;
    //std::cout <<"roi angle = " << roi.angle << std::endl;
    return true;
}

bool DetectionByFeature::isRectangle(double x1, double y1,
                 double x2, double y2,
                 double x3, double y3,
                 double x4, double y4)
{
    double cx,cy;
    double dd1,dd2,dd3,dd4;

    cx=(x1+x2+x3+x4)/4;
    cy=(y1+y2+y3+y4)/4;

    dd1=sqrt((cx-x1)*(cx-x1)+(cy-y1)*(cy-y1));
    dd2=sqrt((cx-x2)*(cx-x2)+(cy-y2)*(cy-y2));
    dd3=sqrt((cx-x3)*(cx-x3)+(cy-y3)*(cy-y3));
    dd4=sqrt((cx-x4)*(cx-x4)+(cy-y4)*(cy-y4));
    double mean = (dd1 + dd2 + dd3 + dd4)/4;
    double var = ((dd1-mean)*(dd1-mean) + (dd2-mean)*(dd2-mean)
                  +(dd3-mean)*(dd3-mean) + (dd4-mean)*(dd4-mean))/4;
    //std::cout<<"mean = " << mean << " var = " << var << std::endl;
    if (25 * sqrt(var) > mean)
        return false;
    else
        return true;
}

void DetectionByFeature::objects_sub_callback(const std_msgs::Float32MultiArray &msg)
{
    if (msg.data.empty())
        return;
    // the id of the object
    // int id = (int)msg.data[0];

    float ow = msg.data[1];
    float oh = msg.data[2];

    H.at<float>(0,0) = msg.data[3];
    H.at<float>(1,0) = msg.data[4];
    H.at<float>(2,0) = msg.data[5];

    H.at<float>(0,1) = msg.data[6];
    H.at<float>(1,1) = msg.data[7];
    H.at<float>(2,1) = msg.data[8];

    H.at<float>(0,2) = msg.data[9];
    H.at<float>(1,2) = msg.data[10];
    H.at<float>(2,2) = msg.data[11];

    std::vector<cv::Point2f> obj_corners(4);
    obj_corners[0] = cv::Point2f(0.0, 0.0);
    obj_corners[1] = cv::Point2f(ow, 0.0);
    obj_corners[2] = cv::Point2f(ow, oh);
    obj_corners[3] = cv::Point2f(0.0, oh);
    scene_corners.clear();
    cv::perspectiveTransform(obj_corners, scene_corners, H);

}
