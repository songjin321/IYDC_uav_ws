//
// Created by songjin on 18-5-22.
//

#include "detection/DetectionAndTrackingLoop.h"
#include <opencv2/tracking.hpp>

DetectionAndTrackingLoop::DetectionAndTrackingLoop(DetectionBase *p_detector) :
        state(State::wait),detector(p_detector)
{
    tracker = cv::TrackerKCF::create();
    std::cout << "wait for start signal" << std::endl;
}
DetectionAndTrackingLoop::DetectionAndTrackingLoop() :
        state(State::wait),detector(nullptr)
{
    tracker = cv::TrackerKCF::create();
    std::cout << "wait for start signal" << std::endl;
};
void DetectionAndTrackingLoop::setDetector(DetectionBase *p_detector)
{
    detector = p_detector;
}
DetectionAndTrackingLoop::~DetectionAndTrackingLoop()
{
    delete detector;
}

bool DetectionAndTrackingLoop::detectFrame(cv::Mat &frame, cv::Rect2d &box)
{
    if (state == detection)
    {
        cv::RotatedRect r_box;
        if(detector->detect(frame, r_box))
        {
            tracker->clear();
            tracker = cv::TrackerKCF::create();
            if(tracker->init(frame, r_box.boundingRect2f()))
            {
                state = tracking;
                std::cerr << "tracker init success, try to tracking" << std::endl;
                return true;
            } else{
                std::cerr << "tracker init failed" << std::endl;
            }
        }
    }
    if (state == tracking)
    {
        if(!tracker->update(frame, box))
        {
            if(is_car_using_feature)
            {
                state = detection;
                std::cerr << "track lost, begin detect Object" << std::endl;
            } else{
                state = wait;
                std::cerr << "track lost, because detection by color, so it will not restart";
            }

        } else{
            // std::cout << "tracking......" << std::endl;
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

