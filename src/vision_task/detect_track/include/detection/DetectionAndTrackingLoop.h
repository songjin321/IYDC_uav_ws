//
// Created by songjin on 18-5-22.
//

#ifndef UAV_WS_DETECTIONANDTRACKINGLOOP_H
#define UAV_WS_DETECTIONANDTRACKINGLOOP_H

#include <detection/DetectionBase.h>
#include <opencv2/tracking.hpp>

/**
 * 状态模式
 */
class DetectionAndTrackingLoop
{
public:
    enum State{wait, detection, tracking};
    DetectionAndTrackingLoop(DetectionBase *detector);
    ~DetectionAndTrackingLoop();
    bool detectFrame(cv::Mat &frame, cv::Rect2d &box);
    void setState(State s);
private:
    State state;
    DetectionBase *detector;
    cv::Ptr<cv::Tracker> tracker;
};
#endif //UAV_WS_DETECTIONANDTRACKINGLOOP_H
