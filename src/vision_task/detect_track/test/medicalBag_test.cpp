
#include <ros/ros.h>
#include "ros_common/RosImageToMat.h"
#include "detection/DetectionByFeature.h"
#include <opencv2/core/utility.hpp>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "medicalBag_test");
    ros::NodeHandle nh;
    RosImageToMat imageToMat("/camera/image_raw", nh);
    DetectionByFeature detector("/home/songjin/Project/uav_ws/save_floder/objects/5.png");
    cv::Mat frame;
    cv::Rect2d box;
    while(ros::ok())
    {
        ros::spinOnce();
        if (!imageToMat.getNewImage(frame))
            continue;
        // 不是物体也会被检测出来？？
        if(detector.detect(frame, box))
            cv::rectangle(frame, box, CV_RGB(255,0,0));
        cv::imshow("medica detection", frame);
        cv::waitKey(3);
    }
}