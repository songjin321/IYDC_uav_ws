//
// Created by songjin on 18-5-22.
//

#include "detection/DetectionAndTrackingLoop.h"
#include <opencv2/tracking.hpp>

DetectionAndTrackingLoop::DetectionAndTrackingLoop(DetectionBase *detector, cv::Tracker *tracker):
        state(State::wait),detector(detector),tracker(tracker)
{

}

DetectionAndTrackingLoop::~DetectionAndTrackingLoop()
{
    delete detector;
    delete tracker;
}

cv::Rect2f DetectionAndTrackingLoop::detectFrame(cv::Mat &frame)
{
    cv::Rect2d roi = cv::Rect2d(0,0,0,0);
    if (state == wait)
    {
        return roi;
    }
    if (state == detection)
    {
        if(detector->detect(frame, roi))
        {
            return roi;
            tracker->init(frame, roi);
            state = tracking;
        }
    }
    if (state == tracking)
    {
        if(!tracker->update(frame, roi))
            state = detection;
    }
    return roi;
}

void DetectionAndTrackingLoop::setState(DetectionAndTrackingLoop::State s) {
    state = s;
    std::cout << "state = " << s << std::endl;
}
