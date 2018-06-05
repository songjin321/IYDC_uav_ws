//
// Created by songjin on 18-5-22.
//

#ifndef UAV_WS_DETECTIONBYCOLOR_H
#define UAV_WS_DETECTIONBYCOLOR_H

#include <detection/DetectionBase.h>

class DetectionByColor:DetectionBase
{
public:
    DetectionByColor(uchar low_hue, uchar high_hue);
    bool detect(cv::Mat &sceneImg, cv::Rect2d &roi) override;
    bool detect(cv::Mat &sceneImg, cv::RotatedRect &roi) override;
private:
    uchar low_hue_;
    uchar high_hue_;
};
#endif //UAV_WS_DETECTIONBYCOLOR_H
