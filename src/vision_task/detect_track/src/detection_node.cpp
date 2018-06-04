//
// Created by songjin on 18-6-1.
//
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
#include "detection/DetectionByColor.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detectMedicalBag");
    ros::NodeHandle nh;
    // get some parameters from parameter servers
    // HSV threshold in color person detection

    // capture image
    RosImageToMat imageToMat("/camera/image_raw", nh);

    // calculate object pose relative camera
    ObjectPoseCal object_pose("/camera/info","medicalBag_pose");

    // detectors
    std::string parent_path = "../../share/detect_track/";
    DetectionByFeature medicalBag_detector(parent_path+"objects/medicalBag.png");
    DetectionByFeature car_detector(parent_path+"objects/car.png");
    DetectionByColor bluePerson_detector();
    DetectionAndTrackingLoop car_dAt(&car_detector);

    // detection controller, server
    DetectionController detection_controller(&car_dAt);
    ros::ServiceServer service = nh.advertiseService("detection_controller_server",
                                                     &DetectionController::controlDetectionCallback,
                                                     &detection_controller);

    cv::Mat frame;
    cv::Rect2d box;
    while(ros::ok())
    {
        ros::spinOnce();
        if (!imageToMat.getNewImage(frame))
            continue;
        if(detection_controller.is_detect_car_)
        {
            if(car_dAt.detectFrame(frame, box))
            {
                object_pose.calculatePoseFromBox(box);
                //object_pose.publishCarPose();
                cv::rectangle(frame, box, CV_RGB(255,0,0));
            }
        }
        if(detection_controller.is_detect_medicalBag_)
        {
            medicalBag_detector.detect(frame, box);
            //object_pose.publishMedicalBagPose();
        }
        if(detection_controller.is_detect_bluePerson_)
        {

        }
        cv::imshow("detection_result", frame);
        cv::waitKey(3);
    }
}
