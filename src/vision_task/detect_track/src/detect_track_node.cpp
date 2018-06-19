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
    DetectionByColor color_detector;
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
    ros::Rate rate(30);
    while(ros::ok())
    {
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
            if(detection_controller.is_detect_BackgroundObject_)
            {
                // 最大的轮廓
                cv::RotatedRect r_box_1;
                // 第二大的轮廓
                cv::RotatedRect r_box_2;
                bool is_detect_object = color_detector.detectBackgroundObject(frame, r_box_1, r_box_2,
                                                                              cv::Scalar(40,0,0),
                                                                              cv::Scalar(80,255,255));
                // 计算目标物的位置
                if(is_detect_object)
                {
                    ROS_INFO("detected backgroundObject!!!");
                    object_pose.calculatePoseFromRotatedBox(r_box_2);
                    object_pose.publishPose();
                    cv::Point2f vertices[4];
                    r_box.points(vertices);
                    for (int i = 0; i < 4; i++)
                        line(frame, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));
                }else
                {
                    // 一个轮廓都没有检测到,无法计算目标物的位置
                    // 最大的轮廓不为空,确定目标物在最大轮廓的那个方向
                    // 如果视野中全是背景色,无法确定目标物的位置

                    if(r_box_1.size.area() != 0 && abs(r_box_1.size.area() - (frame.rows + frame.cols)*2) > 200)
                    {
                        ROS_INFO("detected background!!!");
                        object_pose.calculatePoseFromRotatedBox(r_box_1);
                        object_pose.publishPose();
                        cv::Point2f vertices[4];
                        r_box.points(vertices);
                        for (int i = 0; i < 4; i++)
                            line(frame, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));
                    }
                }
            }
            if(detection_controller.is_detect_redPerson_)
            {
                if(color_detector.detectPureObject(frame, r_box,
                                                cv::Scalar(0, 50, 50),
                                                cv::Scalar(10, 255, 255),
                                                cv::Scalar(170, 50, 50),
                                                cv::Scalar(180, 255, 255)))
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
            if(detection_controller.is_detect_yellowPerson_)
            {
                if(color_detector.detectPureObject(frame, r_box,
                                                cv::Scalar(0, 50, 50),
                                                cv::Scalar(10, 255, 255)))
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
                if(color_detector.detectBlackCircle(frame, black_center))
                {
                    // ROS_INFO("detected colorPerson!!!");
                    object_pose.calculatePoseFromPoint(black_center);
                    object_pose.publishPose();
                    cv::circle( frame, black_center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
                }
            }
            //cv::imshow("detection_result", frame);
            //cv::waitKey(3);
        }
        rate.sleep();
    }
}
