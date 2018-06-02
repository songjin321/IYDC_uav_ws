//
// Created by songjin on 18-5-22.
//

#include "ros_common/RosImageToMat.h"
#include "detection/DetectionByColor.h"
#include "../src/detection/DetectionAndTrackingLoop.cpp"
#include "detection/ObjectPosePub.h"
int main(int argc, char** argv)
{
    ros::init(argc, argv, "detectMedicalBag");
    RosImageToMat imageToMat("/camera/image_raw");
    ObjectPosePub object_pose_pub("medicalBag_box");
    DetectionByColor detector();
    cv::Ptr<cv::TrackerKCF> tracker = cv::TrackerKCF::create();
    DetectionAndTrackingLoop  At(&detector, tracker);
    while(ros::ok())
    {
        imageToMat.getImage(frame);
        object_pose_pub.publish(dAt.detectFrame(frame));
        ros::spinOnce();
    }
}
