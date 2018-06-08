//
// Created by songjin on 18-5-22.
//
#include "detection/DetectionByColor.h"
#include <opencv2/opencv.hpp>
DetectionByColor::DetectionByColor(uchar low_hue, uchar high_hue):
low_hue_(low_hue), high_hue_(high_hue)
{
}
bool DetectionByColor::detect(cv::Mat &sceneImg, cv::Rect2d &roi)
{
    cv::Mat hsvImg;
    cv::cvtColor(sceneImg, hsvImg, CV_BGR2HSV);
    cv::Scalar hsv_l(low_hue_,50,50);
    cv::Scalar hsv_h(high_hue_,255,255);
    cv::Mat bw;
    inRange(hsvImg, hsv_l, hsv_h, bw);
    // close operation
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5,5));
    cv::morphologyEx(bw, bw, cv::MORPH_CLOSE, element);
    imshow("Specific Colour", bw);
    return false;
}

bool isSmaller(const std::vector<cv::Point> &s1, const std::vector<cv::Point> &s2)
{
    return s1.size() < s2.size();
}

bool DetectionByColor::detect(cv::Mat &sceneImg, cv::RotatedRect &roi)
{
    cv::Mat hsvImg;
    cv::cvtColor(sceneImg, hsvImg, CV_BGR2HSV);
    cv::Scalar hsv_l(low_hue_,50,50);
    cv::Scalar hsv_h(high_hue_,255,255);
    cv::Mat bw;
    inRange(hsvImg, hsv_l, hsv_h, bw);
    // imshow("Specific Colour", bw);
    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bw, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point());

    // sort by size in contours
    // std::cout << "find contours number = " << contours.size() << std::endl;
    if(contours.size() < 2)
        return false;
    std::stable_sort(contours.begin(), contours.end(), isSmaller);
    //
    roi = cv::minAreaRect(*(contours.end()-2));
    return true;
}


