//
// Created by songjin on 18-6-1.
//

#ifndef DETECT_TRACK_DETECTIONCONTROLLER_H
#define DETECT_TRACK_DETECTIONCONTROLLER_H

#include "DetectionAndTrackingLoop.h"
#include <ros/ros.h>
#include "detect_track/ControlDetection.h"
#include <string>

class DetectionController
{
public:
    DetectionController(DetectionAndTrackingLoop *car_dAt);
    bool controlDetectionCallback(detect_track::ControlDetection::Request &req,
                                  detect_track::ControlDetection::Response &res);
    bool is_detect_car_;
    bool is_detect_medicalBag_;
    bool is_detect_BackgroundObject_;
    bool is_detect_redPerson_;
    bool is_detect_blackCircle_;
    bool is_detect_yellowPerson_;

private:
    DetectionAndTrackingLoop *car_dAt_;
};


#endif //DETECT_TRACK_DETECTIONCONTROLLER_H
