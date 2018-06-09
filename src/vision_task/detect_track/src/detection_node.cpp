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
    ros::init(argc, argv, "detect_track_node");
    ros::NodeHandle nh;
    // get some parameters from parameter servers
    // HSV threshold in color person detection

    //

    // capture image
    RosImageToMat imageToMat("/usb_cam/image_rect_color", nh);

    // calculate object pose relative camera
    ObjectPoseCal object_pose("/usb_cam/camera_info","object_pose");

    // detectors
    DetectionByFeature medicalBag_detector;
    DetectionByFeature car_detector;

    // green is 1, others is 0
    DetectionByColor colorPerson(40, 80);
    DetectionAndTrackingLoop car_dAt(&car_detector);

    // detection controller, server
    DetectionController detection_controller(&car_dAt);
    ros::ServiceServer service = nh.advertiseService("detection_controller_server",
                                                     &DetectionController::controlDetectionCallback,
                                                     &detection_controller);
    cv::Mat frame;
    cv::Rect2d box;
    cv::RotatedRect r_box;
    cv::Point2f black_center;
    while(ros::ok())
    {
        ros::Rate rate(30);
        ros::spinOnce();
        if (imageToMat.getNewImage(frame))
        {
            if(detection_controller.is_detect_car_)
            {
                if(car_dAt.detectFrame(frame, box))
                {
                    // 如果检测成功,必定会发布object_pose
                    object_pose.calculatePoseFromBox(box);
                    object_pose.publishPose();
                    cv::rectangle(frame, box, CV_RGB(255,0,0));
                }
            }
            if(detection_controller.is_detect_medicalBag_)
            {
                if(medicalBag_detector.detect(frame, r_box))
                {
                    // ROS_INFO("detected medicalBag!!!");
                    object_pose.calculatePoseFromRotatedBox(r_box);
                    object_pose.publishPose();
                    // cv::Point2f *vertices = &medicalBag_detector.scene_corners[0];
                    cv::Point2f vertices[4];
                    r_box.points(vertices);
                    for (int i = 0; i < 4; i++)
                        line(frame, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));
                }
            }
            if(detection_controller.is_detect_colorPerson_)
            {
                if(colorPerson.detect(frame, r_box))
                {
                    // ROS_INFO("detected colorPerson!!!");
                    object_pose.calculatePoseFromRotatedBox(r_box);
                    object_pose.publishPose();
                    cv::Point2f vertices[4];
                    r_box.points(vertices);
                    for (int i = 0; i < 4; i++)
                        line(frame, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));
                }
            }
            if(detection_controller.is_detect_redPerson_)
            {
                if(colorPerson.detectRedPerson(frame, r_box))
                {
                    // ROS_INFO("detected colorPerson!!!");
                    object_pose.calculatePoseFromRotatedBox(r_box);
                    object_pose.publishPose();
                    cv::Point2f vertices[4];
                    r_box.points(vertices);
                    for (int i = 0; i < 4; i++)
                        line(frame, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));
                }
            }
            if(detection_controller.is_detect_blackCircle_)
            {
                if(colorPerson.detectBlackCircle(frame, black_center))
                {
                    // ROS_INFO("detected colorPerson!!!");
                    object_pose.calculatePoseFromPoint(black_center);
                    object_pose.publishPose();
                    cv::circle( frame, black_center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
                }
            }
            cv::imshow("detection_result", frame);
            cv::waitKey(3);
        }
        rate.sleep();
    }
}
