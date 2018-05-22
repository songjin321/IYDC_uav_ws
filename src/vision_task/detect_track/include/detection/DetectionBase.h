//
// Created by songjin on 18-5-22.
//

#ifndef UAV_WS_DETECTIONBASE_H
#define UAV_WS_DETECTIONBASE_H
class DetectionBase
{
    void detect(cv::Mat &sceneImg, cv::Rect2f &roi) = 0;
};
#endif //UAV_WS_DETECTIONBASE_H
