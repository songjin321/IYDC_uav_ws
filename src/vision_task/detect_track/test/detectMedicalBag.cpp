//
// Created by songjin on 18-5-22.
//
#include <ros/ros.h>
#include "ros_common/RosImageToMat.h"
#include "detection/DetectionByFeature.h"
#include "detection/DetectionAndTrackingLoop.h"
#include "detection/DetectionController.h"
#include "detection/ObjectPoseCal.h"
#include <opencv2/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include "detect_track/ControlDetection.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detectMedicalBag");
    ros::NodeHandle nh;
    RosImageToMat imageToMat("/camera/image_raw", nh);
    ObjectPoseCal object_pose("/camera/info","medicalBag_pose");
    DetectionByFeature detector("/home/songjin/Project/uav_ws/save_floder/objects/6.png");
    DetectionAndTrackingLoop dAt(&detector);
    ros::ServiceServer service = nh.advertiseService<detect_track::ControlDetection::Request,
            detect_track::ControlDetection::Response>
            ("control_detection_server", boost::bind(control_detection, _1, _2, &dAt));
    cv::Mat frame;
    cv::Rect2d box;
    while(ros::ok())
    {
        ros::spinOnce();
        if (!imageToMat.getNewImage(frame))
            continue;
        if(dAt.detectFrame(frame, box))
            cv::rectangle(frame, box, CV_RGB(255,0,0));
        cv::imshow("medica detection", frame);
        cv::waitKey(3);
    }
}
