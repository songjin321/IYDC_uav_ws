//
// Created by songjin on 18-6-4.
//

#include "detection/DetectionController.h"
#include <stdlib.h>
DetectionController::DetectionController(DetectionAndTrackingLoop* p_car_dAt):
detection_type_(DetectionType::None),car_dAt_(p_car_dAt)
{
}
bool DetectionController::controlDetectionCallback(detect_track::ControlDetection::Request &req,
                                                   detect_track::ControlDetection::Response &res)
{
    if (req.ControlType < 0 || req.ControlType > 7)
    {
        ROS_INFO("no correspond detection type");
        res.setOk = false;
        return false;
    }
    detection_type_ = static_cast<DetectionType>(req.ControlType);
    ROS_INFO("detection type = %d", static_cast<int>(detection_type_));
    if(detection_type_ == DetectionType::Car)
    {
        car_dAt_->beginDetection();
        ROS_INFO("car detection begin");
        if (car_dAt_->is_car_using_feature)
            system("roslaunch competition_tasks find_object.launch object_name:=car &");
    }
    res.setOk = true;
    return true;
}
