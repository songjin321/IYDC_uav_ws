//
// Created by songjin on 18-5-22.
//

#include "common/RosImageToMat.h"
#include "detection/DetectionByColor.h"
#include "detection/DetectionAndTrackingLoop.cpp"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detectMedicalBag");
    RosImageToMat imageToMat("/camera/image_raw");
    RosMessageSubAndPub object_box_pub;
    DetectionByFeature detector("./objects/5.png");
    std::shared_ptr<cv::Tracker> tracker = cv::Tracker::create("KCF");
    DetectionAndTrackingLoop dAt(detector, tracker);
    while(ros::ok())
    {
        imageToMat.getImage(frame);
        object_box_pub.publish(dAt.detectFrame(frame));
        ros::spinOnce();
    }
}
