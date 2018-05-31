//
// Created by songjin on 18-5-22.
//

#ifndef UAV_WS_DETECTIONBYCOLOR_H
#define UAV_WS_DETECTIONBYCOLOR_H

#include <detection/DetectionBase.h>

class DetectionByColor:DetectionBase
{
    DetectionByColor();
    bool detect(cv::Mat &sceneImg, cv::Rect2d &roi) override;
};
#endif //UAV_WS_DETECTIONBYCOLOR_H
