//
// Created by songjin on 18-5-22.
//
#include "detection/DetectionByColor.h"
#include <opencv2/opencv.hpp>
DetectionByColor::DetectionByColor(uchar low_hue, uchar high_hue):
low_hue_(low_hue), high_hue_(high_hue)
{
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
    imshow("Specific Colour", bw);
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

bool DetectionByColor::detectBlackCircle(cv::Mat &sceneImg, cv::Point2f &center)
{
    cv::RotatedRect r_box;
    detect(sceneImg, r_box);
    cv::Mat imCrop = sceneImg(r_box.boundingRect());
    cv::Mat src_gray;
    /// Convert it to gray
    cvtColor( imCrop, src_gray, cv::COLOR_BGR2GRAY );

    /// Reduce the noise so we avoid false circle detection
    GaussianBlur( src_gray, src_gray, cv::Size(9, 9), 2, 2 );

    std::vector<cv::Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    HoughCircles( src_gray, circles, cv::HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
    std::cout << "circle number: " << circles.size() <<std::endl;

    if (circles.empty())
        return false;

    /// calculate the average center
    cv::Point2f circle_center_sum(0.0, 0.0);
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point2f center_(cvRound(circles[i][0]), cvRound(circles[i][1]));
        circle_center_sum = circle_center_sum + center_;
    }

    center.x = circle_center_sum.x/circles.size() + r_box.boundingRect().x,
    center.y = circle_center_sum.y/circles.size() + r_box.boundingRect().y;
    return true;
}

bool DetectionByColor::detectRedPerson(cv::Mat &sceneImg, cv::RotatedRect &roi)
{
    cv::Mat hsvImg;
    cv::cvtColor(sceneImg, hsvImg, CV_BGR2HSV);
    cv::Mat mask1, mask2;
    inRange(hsvImg, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), mask1);
    inRange(hsvImg, cv::Scalar(170, 50, 50), cv::Scalar(180, 255, 255), mask2);
    cv::Mat bw = mask1 | mask2;

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bw, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point());
    std::stable_sort(contours.begin(), contours.end(), isSmaller);
    //
    roi = cv::minAreaRect(*(contours.end()-1));
}
