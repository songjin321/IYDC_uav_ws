//
// Created by songjin on 18-5-22.
//

#ifndef UAV_WS_DETECTIONBYFEATURE_H
#define UAV_WS_DETECTIONBYFEATURE_H

#include <string>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <vector>

class DetectionByFeature:DetectionBase
{
public:
    DetectionByFeature(path_object);
    ~DetectionByFeature()
    {
        delete detector;
    };

    bool detect(cv::Mat &sceneImg, cv::Rect2f &roi);
private:
    std::string path_object_;
    std::vector<cv::KeyPoint> objectKeypoints;
    std::vector<cv::KeyPoint> sceneKeypoints;
    cv::Mat objectDescriptors;
    cv::Mat sceneDescriptors;
    cv::FeatureDetector *detector;
    cv::DescriptorExtractor * extractor;
    cv::Mat H;
    cv::Rect2d box;
    int object_width;
    int object_height;

    void initObject();

    /*
     * FLANN -> H
     */
    void computerH();

    /*
     * H and ImageSize -> Box
     */
    void computerBox();

    /*
     * return box
     */
    void getBox(cv::Rect2f &roi);
};
#endif //UAV_WS_DETECTIONBYFEATURE_H
