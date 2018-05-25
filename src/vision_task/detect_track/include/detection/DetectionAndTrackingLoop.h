//
// Created by songjin on 18-5-22.
//

#ifndef UAV_WS_DETECTIONANDTRACKINGLOOP_H
#define UAV_WS_DETECTIONANDTRACKINGLOOP_H

#include <detection/DetectionBase.h>
#include <opencv2/tracking.hpp>

class DetectionAndTrackingLoop
{
public:
    enum State{wait, detection, tracking};
    DetectionAndTrackingLoop(DetectionBase *detector, cv::Tracker * tracker);
    ~DetectionAndTrackingLoop();
    cv::Rect2f detectFrame(cv::Mat &frame);
    void setState(State s);
private:
    State state;
    cv::Mat frame;
    DetectionBase *detector;
    cv::Tracker *tracker;
};
#endif //UAV_WS_DETECTIONANDTRACKINGLOOP_H
