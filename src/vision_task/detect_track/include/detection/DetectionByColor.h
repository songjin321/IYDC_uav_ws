//
// Created by songjin on 18-5-22.
//

#ifndef UAV_WS_DETECTIONBYCOLOR_H
#define UAV_WS_DETECTIONBYCOLOR_H

#include <detection/DetectionBase.h>

class DetectionByColor
{
public:
    DetectionByColor(uchar low_hue, uchar high_hue);
    bool detect(cv::Mat &sceneImg, cv::RotatedRect &roi);
    bool detectRedPerson(cv::Mat &sceneImg, cv::RotatedRect &roi);
    bool detectBlackCircle(cv::Mat &sceneImg, cv::Point2f &center);
private:
    uchar low_hue_;
    uchar high_hue_;
};
#endif //UAV_WS_DETECTIONBYCOLOR_H
