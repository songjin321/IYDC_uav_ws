//
// Created by songjin on 18-6-1.
//

#ifndef DETECT_TRACK_DETECTIONCONTROLLER_H
#define DETECT_TRACK_DETECTIONCONTROLLER_H

#include "DetectionAndTrackingLoop.h"
#include <ros/ros.h>
#include "detect_track/ControlDetection.h"
#include <string>

/**
 * 提供一个能控制检测开始和结束的服务
 */
bool control_detection(detect_track::ControlDetection::Request &req,
                       detect_track::ControlDetection::Response &res,
                       DetectionAndTrackingLoop *p_dAt_)
{
    if(req.ControlType == 0)
    {
        p_dAt_->setState(DetectionAndTrackingLoop::wait);
        res.isOk = true;
    }
    if(req.ControlType == 1)
    {
        p_dAt_->setState(DetectionAndTrackingLoop::detection);
        res.isOk = true;
    }
    return true;
}
#endif //DETECT_TRACK_DETECTIONCONTROLLER_H
