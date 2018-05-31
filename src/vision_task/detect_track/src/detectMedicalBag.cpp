//
// Created by songjin on 18-5-22.
//
#include <ros/ros.h>
#include "ros_common/RosImageToMat.h"
#include "std_msgs/Float32MultiArray.h"
#include "detection/DetectionByFeature.h"
#include "detection/DetectionAndTrackingLoop.h"
#include "detect_track/ControlDetection.h"
#include <opencv2/tracking.hpp>
#include <opencv2/core/utility.hpp>
#include "detection/ObjectPosePub.h"

class ControlDetection
{
public:
    ControlDetection(DetectionAndTrackingLoop *p_dAt)
    :p_dAt_(p_dAt)
    {
        ros::ServiceServer service = nh.advertiseService("control_detection", &ControlDetection::control_detection,
                                                         this);
    }
    bool control_detection(detect_track::ControlDetection::Request &req,
                      detect_track::ControlDetection::Response &res)
    {
        if(req.ControlType == 0)
        {
            p_dAt_->setState(DetectionAndTrackingLoop::wait);
        }
        if(req.ControlType == 1)
        {
            p_dAt_->setState(DetectionAndTrackingLoop::detection);
        }
        return true;
    }

private:
    DetectionAndTrackingLoop* p_dAt_;
    ros::NodeHandle nh;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "detectMedicalBag");
    ros::NodeHandle nh;
    RosImageToMat imageToMat("/camera/image_raw");
    ObjectPosePub object_pose_pub("medicalBag_box");
    DetectionByFeature detector("./objects/5.png");

    // create a tracker object

    cv::Ptr<cv::TrackerKCF> tracker = cv::TrackerKCF::create();
    DetectionAndTrackingLoop dAt(&detector, tracker);
    ControlDetection detection_controler(DetectionAndTrackingLoop *p_dAt);
    cv::Mat frame;
    while(ros::ok())
    {
        imageToMat.getImage(frame);
        object_pose_pub.publish(dAt.detectFrame(frame));
        ros::spinOnce();
    }
}