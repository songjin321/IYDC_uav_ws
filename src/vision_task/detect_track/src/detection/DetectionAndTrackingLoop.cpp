//
// Created by songjin on 18-5-22.
//

#include "DetectionAndTrackingLoop.h"


DetectionAndTrackingLoop::DetectionAndTrackingLoop(DetectionBase *detector, cv::Tracker *tracker):
        state(State::wait),detector(detector),tracker(tracker)
{

}
cv::Rect2f DetectionAndTrackingLoop::detectFrame(cv::Mat &frame)
{
    // TODO::get a signal, begin detection
    if (TODO)
        state = detection;
    if (state == wait)
        continue;
    if (state == detection)
    {
        if(detector.detect(frame, roi))
        {
            tracker->init(frame, roi);
            state = tracking;
        }
    }
    if (state == tracking)
    {
        if(!tracker->update(frame, roi))
            state == detection;
    }
    return roi;
}
