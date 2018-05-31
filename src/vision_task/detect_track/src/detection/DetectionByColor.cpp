//
// Created by songjin on 18-5-22.
//
#include "detection/DetectionByColor.h"
#include <opencv2/opencv.hpp>
bool DetectionByColor::detect(cv::Mat &sceneImg, cv::Rect2d &roi)
{
    cv::Mat hsvImg;
    cv::cvtColor(sceneImg, hsvImg, CV_BGR2HSV);
    cv::Scalar hsv_l(50,50,50);
    cv::Scalar hsv_h(70,255,255);
    cv::Mat bw;
    inRange(hsvImg, hsv_l, hsv_h, bw);
    imshow("Specific Colour", bw);
    return false;
}
