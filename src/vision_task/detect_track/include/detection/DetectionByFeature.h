//
// Created by songjin on 18-5-22.
//

#ifndef UAV_WS_DETECTIONBYFEATURE_H
#define UAV_WS_DETECTIONBYFEATURE_H

#include <string>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <vector>
#include <detection/DetectionBase.h>
#include <opencv2/xfeatures2d.hpp>
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
class DetectionByFeature: public DetectionBase
{
public:
    DetectionByFeature(std::string path_object);
    ~DetectionByFeature()
    {
        delete detector;
    };

    bool detect(cv::Mat &sceneImg, cv::Rect2d &roi) override;
    bool detect(cv::Mat &sceneImg, cv::RotatedRect &roi) override;

    void objects_sub_callback(const std_msgs::Float32MultiArray& msg);
    // four corners of the detected object
    std::vector<cv::Point2f> scene_corners;
private:
    std::string path_object_;
    std::vector<cv::KeyPoint> objectKeypoints;
    std::vector<cv::KeyPoint> sceneKeypoints;
    cv::Mat objectDescriptors;
    cv::Mat sceneDescriptors;
    cv::Ptr<cv::xfeatures2d::SIFT> detector;
    cv::Mat H;
    cv::Rect2d box;
    int object_width;
    int object_height;

    void initObject();

    /*
     * FLANN -> H
     */
    bool computerH();

    /*
     * 计算四个顶点
     */
    void computerFourVertex();

    /*
     * H and ImageSize -> Box
     */
    bool computerBox();

    /*
     * judge is it a rectangle
     */
    bool isRectangle(double x1, double y1,
                     double x2, double y2,
                     double x3, double y3,
                     double x4, double y4);

};
#endif //UAV_WS_DETECTIONBYFEATURE_H
