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

class ObjectBoxPub
{
public:
    ObjectBoxPub(const std::string &publish_topic_name)
    {
        //Topic you want to publish
        pub_ = n_.advertise<std_msgs::Float32MultiArray>(publish_topic_name , 1);
    }
    void publish(const cv::Rect2f &box)
    {
        output.data.push_back(box.x);
        output.data.push_back(box.y);
        output.data.push_back(box.width);
        output.data.push_back(box.height);
        pub_.publish(output);
    }
private:
    ros::NodeHandle n_;
    ros::Publisher pub_;
    std_msgs::Float32MultiArray output;
};
int main(int argc, char** argv)
{
    ros::init(argc, argv, "detectMedicalBag");
    ros::NodeHandle nh;
    RosImageToMat imageToMat("/camera/image_raw");
    ObjectBoxPub object_box_pub("medicalBag_box");
    DetectionByFeature detector("./objects/5.png");

    // create a tracker obje

    cv::Ptr<cv::TrackerKCF> tracker = cv::TrackerKCF::create();
    DetectionAndTrackingLoop dAt(&detector, tracker);
    ControlDetection detection_controler(DetectionAndTrackingLoop *p_dAt);
    cv::Mat frame;
    while(ros::ok())
    {
        imageToMat.getImage(frame);
        object_box_pub.publish(dAt.detectFrame(frame));
        ros::spinOnce();
    }
}