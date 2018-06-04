//
// Created by songjin on 18-6-4.
//

#include "detection/DetectionController.h"
DetectionController::DetectionController(DetectionAndTrackingLoop *car_dAt):
        is_detect_car_(false),is_detect_medicalBag_(false),
        is_detect_bluePerson_(false), car_dAt_(car_dAt)
{

}
bool DetectionController::controlDetectionCallback(detect_track::ControlDetection::Request &req,
                                                   detect_track::ControlDetection::Response &res)
{
    switch(req.ControlType)
    {
        case 0:
            if(req.Start)
            {
                car_dAt_->beginDetection();
            } else{
                car_dAt_->stopDetection();
            }
            break;
        case 1:
            is_detect_medicalBag_ = req.Start;
            break;
        case 2:
            is_detect_bluePerson_ = req.Start;
            break;
        default:
            ROS_INFO("no correspond detection type");
            return false;
            break;
    }
    return true;
}