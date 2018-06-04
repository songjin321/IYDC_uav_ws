//
// Created by songjin on 18-5-22.
//

#include "detection/DetectionAndTrackingLoop.h"
#include <opencv2/tracking.hpp>

DetectionAndTrackingLoop::DetectionAndTrackingLoop(DetectionBase *detector) :
        state(State::wait),detector(detector)
{
    tracker = cv::TrackerKCF::create();
    std::cout << "wait for start signal" << std::endl;
}

DetectionAndTrackingLoop::~DetectionAndTrackingLoop()
{
    delete detector;
}

bool DetectionAndTrackingLoop::detectFrame(cv::Mat &frame, cv::Rect2d &box)
{
    if (state == detection)
    {
        if(detector->detect(frame, box))
        {
            std::cout << "detect success, try to init tracker" << std::endl;
            tracker->clear();
            tracker = cv::TrackerKCF::create();
            if(tracker->init(frame, box))
            {
                state = tracking;
                std::cout << "tracker init success" << std::endl;
                return true;
            } else{
                std::cout << "tracker init failed" << std::endl;
            }
        }
    }
    if (state == tracking)
    {
        if(!tracker->update(frame, box))
        {
            state = detection;
            std::cout << "track lost, begin detect" << std::endl;

        } else{
            std::cout << "tracking......" << std::endl;
            return true;
        }
    }
    return false;
}

void DetectionAndTrackingLoop::beginDetection()
{
    state = detection;
}

void DetectionAndTrackingLoop::stopDetection()
{
    state = wait;
}

