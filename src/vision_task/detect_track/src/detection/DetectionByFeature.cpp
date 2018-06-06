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

DetectionByFeature::DetectionByFeature(std::string path_object):
        path_object_(path_object)
{
    // The detector can be any of (see OpenCV features2d.hpp):
    // cv::FeatureDetector * detector = new cv::DenseFeatureDetector();
    // cv::FeatureDetector * detector = new cv::FastFeatureDetector();
    // cv::FeatureDetector * detector = new cv::GFTTDetector();
    // cv::FeatureDetector * detector = new cv::MSER();
    detector = cv::xfeatures2d::SIFT::create();
    // cv::FeatureDetector * detector = new cv::StarFeatureDetector();
    // cv::FeatureDetector * detector = new cv::SURF(600.0);
    // cv::FeatureDetector * detector = new cv::BRISK();
    initObject();
    H = cv::Mat::zeros(3,3, CV_32F);

}
void DetectionByFeature::initObject()
{
    cv::Mat objectImg = cv::imread(path_object_, cv::IMREAD_GRAYSCALE);
    if (objectImg.empty())
    {
        std::cerr << "read object template image failed" << std::endl;
        return;
    }
    object_width = objectImg.cols;
    object_height = objectImg.rows;
    detector->detect(objectImg, objectKeypoints);
    std::cout << "object template feature number: " << objectKeypoints.size() << std::endl;
    detector->compute(objectImg, objectKeypoints, objectDescriptors);
}
bool DetectionByFeature::detect(cv::Mat &sceneImg, cv::Rect2d &roi)
{
    if (scene_corners.empty())
        return false;
    roi = cv::minAreaRect(scene_corners).boundingRect2f();
    std::cout << "box area equal to " << roi.area() <<std::endl;
    if (roi.area() > 100000)
        return false;
    return true;
}
bool DetectionByFeature::detect(cv::Mat &sceneImg, cv::RotatedRect &roi)
{
    if (scene_corners.empty())
        return false;
    roi = cv::minAreaRect(scene_corners);
    //std::cout <<"roi size = " << roi.size << std::endl;
    //std::cout <<"roi center = " << roi.center << std::endl;
    //std::cout <<"roi angle = " << roi.angle << std::endl;
    return true;
}

bool DetectionByFeature::computerBox()
{
    double ow = object_width;
    double oh = object_height;
    double h11 = H.at<double>(0,0);
    double h12 = H.at<double>(0,1);
    double h13 = H.at<double>(0,2);
    double h21 = H.at<double>(1,0);
    double h22 = H.at<double>(1,1);
    double h23 = H.at<double>(1,2);
    double h31 = H.at<double>(2,0);
    double h32 = H.at<double>(2,1);
    double h33 = H.at<double>(2,2);
    // std::cout << h11 << " " << h12 << " "<< h13 << " "<< h21 << " "<< h22 << " "<< h23 << " "<< h31 << " "
    //       << h32 << " "<< h33 << std::endl;
    // coordinate of four vertexs
    double x0 = h13;
    double y0 = h23;
    double x1 = h11 * ow + h13;
    double y1 = h21 * ow + h23;
    double x2 = h12 * oh + h13;
    double y2 = h22 * oh + h23;
    double x3 = h11 * ow + h12 * oh + h13;
    double y3 = h21 * ow + h22 * oh + h23;
    bool is_rectangle = isRectangle(x0,y0,x1,y1,x2,y2,x3,y3);
    if(!is_rectangle)
        return false;
    // std::cout << "x0 = "<< x0 << " x1 = " <<  x1 << " x2 = " <<  x2
    //          << " x3 = " << x3 << " y0 = " << y0 << " y1 = " << y1 << " y2 = " << y2
    //          << " y3 = " << y3 << std::endl;
    // upper left corner and lower right corner
    double xmin = std::min(x0, std::min(x1, std::min(x2, x3)));
    double xmax = std::max(x0, std::max(x1, std::max(x2, x3)));
    double ymin = std::min(y0, std::min(y1, std::min(y2, y3)));
    double ymax = std::max(y0, std::max(y1, std::max(y2, y3)));
    box = cv::Rect2d(xmin, ymin, xmax-xmin, ymax-ymin);
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
