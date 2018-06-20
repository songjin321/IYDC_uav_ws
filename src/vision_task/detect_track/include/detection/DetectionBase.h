//
// Created by songjin on 18-5-22.
//

#ifndef UAV_WS_DETECTIONBASE_H
#define UAV_WS_DETECTIONBASE_H

#include <opencv2/core/core.hpp>
class DetectionBase
{
public:
    virtual bool detect(cv::Mat &sceneImg, cv::RotatedRect &roi) = 0;
};
#endif //UAV_WS_DETECTIONBASE_H
