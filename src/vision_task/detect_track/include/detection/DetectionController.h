//
// Created by songjin on 18-6-1.
//

#ifndef DETECT_TRACK_DETECTIONCONTROLLER_H
#define DETECT_TRACK_DETECTIONCONTROLLER_H

#include "DetectionAndTrackingLoop.h"
#include <ros/ros.h>
#include "detect_track/ControlDetection.h"
#include <string>
enum class DetectionType{
    None = 0,
    bluePerson = 1,
    RedPerson = 2,
    BlackCircle = 3,
    Car = 4,
    MedicalBag = 5,
    YellowPerson = 6
};
class DetectionController
{
public:
    DetectionController(DetectionAndTrackingLoop* p_car_dAt);
    bool controlDetectionCallback(detect_track::ControlDetection::Request &req,
                                  detect_track::ControlDetection::Response &res);
    DetectionType detection_type_;
private:
    DetectionAndTrackingLoop* car_dAt_;
};


#endif //DETECT_TRACK_DETECTIONCONTROLLER_H
