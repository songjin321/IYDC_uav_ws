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
    std::cerr << "0" << std::endl;
    RosImageToMat imageToMat("/camera/image_raw", nh);
    std::cerr << "1" << std::endl;
    ObjectPoseCal object_pose("/camera/info","medicalBag_pose");
    std::cerr << "2" << std::endl;
    DetectionByFeature detector("/home/songjin/Project/uav_ws/save_floder/objects/5.png");
    std::cerr << "3" << std::endl;
    cv::Ptr<cv::TrackerKCF> tracker = cv::TrackerKCF::create();
    std::cerr << "4" << std::endl;
    DetectionAndTrackingLoop dAt(&detector, tracker);
    std::cerr << "5" << std::endl;
    ros::ServiceServer service = nh.advertiseService<detect_track::ControlDetection::Request,
            detect_track::ControlDetection::Response>
            ("control_detection_server", boost::bind(control_detection, _1, _2, &dAt));
    cv::Mat frame;
    while(ros::ok())
    {
        ros::spinOnce();
        // std::cout << "loop once" << std::endl;

        if (!imageToMat.getNewImage(frame))
            continue;
        cv::Rect2f box = dAt.detectFrame(frame);
        std::cout << box.width << std::endl;
        object_pose.calculateFormBox(box);
        //object_pose.publishPose();
        // draw box
        if (!frame.empty())
        {
            cv::rectangle(frame, box, CV_RGB(255,0,0));
            cv::imshow("medica detection", frame);
            cv::waitKey(3);
        }


    }
}
