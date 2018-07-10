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
    DetectionAndTrackingLoop(DetectionBase *p_detector);
    DetectionAndTrackingLoop();
    ~DetectionAndTrackingLoop();
    bool detectFrame(cv::Mat &frame, cv::Rect2d &box);
    void setDetector(DetectionBase *p_detector);
    void beginDetection();
    void stopDetection();
    bool is_car_using_feature;
private:
    State state;
    DetectionBase *detector;
    cv::Ptr<cv::Tracker> tracker;
    cv::Ptr<cv::Tracker> tracker_color;
};
#endif //UAV_WS_DETECTIONANDTRACKINGLOOP_H
