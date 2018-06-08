//
// Created by songjin on 18-5-22.
//
#include "detection/DetectionByColor.h"
#include <opencv2/opencv.hpp>
int main(int argc, char **argv)
{
    // green is 1, others is 0
    DetectionByColor colorPerson(40, 80);
    cv::Mat srcImage = cv::imread("/home/songjin/Project/uav_ws/exp_data/task2_circle.png");
    cv::Mat hsvImg;
    cv::cvtColor(srcImage, hsvImg, CV_BGR2HSV);
    cv::Scalar hsv_l(0,0,0);
    cv::Scalar hsv_h(180,255,30);
    cv::Mat bw;
    inRange(hsvImg, hsv_l, hsv_h, bw);
    imshow("Specific Colour", bw);
    //cv::Point2f vertices[4];
    //r_box.points(vertices);
    //for (int i = 0; i < 4; i++)
    //    line(srcImage, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));
    cv::imshow("test_result", srcImage);
    cv::waitKey(0);
    return 0;
}