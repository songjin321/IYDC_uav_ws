//
// Created by songjin on 18-6-4.
//

#include "detection/DetectionController.h"
#include <stdlib.h>
DetectionController::DetectionController(DetectionAndTrackingLoop *car_dAt):
        is_detect_car_(false),is_detect_medicalBag_(false),is_detect_blackCircle_(false),
        is_detect_BackgroundObject_(false), car_dAt_(car_dAt), is_detect_yellowPerson_(false),
	is_detect_redPerson_(false)
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
                ROS_INFO("car detection begin");
            } else{
                car_dAt_->stopDetection();
                ROS_INFO("car detection stop");
            }
            is_detect_car_ = req.Start;
            break;
        case 1:
            if(req.Start)
            {
                is_detect_medicalBag_ = true;
                system("roslaunch competition_tasks find_object.launch");
            } else
            {
                is_detect_medicalBag_ = false;
                system("kill find-object-2d");
            }
            ROS_INFO("medical bag detection state: %d", req.Start);
            break;
        case 2:
            is_detect_BackgroundObject_ = req.Start;
            ROS_INFO("Background object detection state: %d", req.Start);
            break;
        case 3:
            is_detect_redPerson_ = req.Start;
            ROS_INFO("red person detection state: %d", req.Start);
            break;
        case 4:
            is_detect_blackCircle_ = req.Start;
            ROS_INFO("black Circle detection state: %d", req.Start);
            break;
        case 5:
            is_detect_yellowPerson_ = req.Start;
            ROS_INFO("yellow person detection state: %d", req.Start);
            break;
        default:
            ROS_INFO("no correspond detection type");
            return false;
            break;
    }
    return true;
}
