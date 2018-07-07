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

int main(int argc, char **argv) {
    ros::init(argc, argv, "detect_track_node");
    ros::NodeHandle nh;
    // get some parameters from parameter servers
    // HSV threshold in color person detection
    // nh.param<double>("/detect_track_node/x_cam2body", x_cam2body, 0);
    // nh.param<double>("/detect_track_node/y_cam2body", y_cam2body, 0);

    // hue and saturation value of green background
    int h_low_greenBG;
    int h_high_greenBG;
    nh.param<int>("/detect_track_node/h_low_greenBG", h_low_greenBG, 40);
    nh.param<int>("/detect_track_node/h_high_greenBG", h_high_greenBG, 80);

    // capture image
    RosImageToMat imageToMat("/usb_cam/image_rect_color", nh);

    // calculate object pose relative camera
    ObjectPoseCal object_pose("/usb_cam/camera_info", "object_pose");

    // detectors
    DetectionByFeature medicalBag_detector;
    DetectionByFeature car_detector;
    DetectionByColor color_detector;

    //
    DetectionAndTrackingLoop car_dAt(&car_detector);

    // detection controller, server
    DetectionController detection_controller;

    ros::ServiceServer service = nh.advertiseService("detection_controller_server",
                                                     &DetectionController::controlDetectionCallback,
                                                     &detection_controller);
    cv::Mat frame;
    cv::Mat hsv;
    cv::Rect2d box;
    cv::RotatedRect r_box;
    cv::RotatedRect r_box_1;
    cv::Point2f black_center;
    bool is_detect_object;
    ros::Rate rate(20);
    while (ros::ok()) {
        ros::spinOnce();
        if (imageToMat.getNewImage(frame)) {
            switch (detection_controller.detection_type_) {
                case DetectionType::None:
                    car_dAt.stopDetection();
                    system("pkill find_object_2d");
                    ROS_INFO("all detection stopped");
                    break;
                case DetectionType::bluePerson:
                    // 检测最大和次大轮廓
                    is_detect_object = color_detector.detectBackgroundObject(frame, r_box_1, r_box,
                                                                             cv::Scalar(h_low_greenBG, 50, 50),
                                                                             cv::Scalar(h_high_greenBG, 255, 255));
                    // 计算目标物的位置
                    if (is_detect_object) {
                        //ROS_INFO("detected blue person!!!");
                        object_pose.calculatePoseFromRotatedBox(r_box);
                        object_pose.publishPose();
                        cv::Point2f vertices[4];
                        r_box.points(vertices);
                        for (int i = 0; i < 4; i++)
                            line(frame, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0));
                    } else if (r_box_1.size.area() > 5000 &&
                               abs(r_box_1.size.area() - (frame.rows * frame.cols)) > 3000) {
                        //ROS_INFO("detected green background!!!");
                        object_pose.calculatePoseFromRotatedBox(r_box_1);
                        object_pose.publishPose();
                        cv::Point2f vertices[4];
                        r_box_1.points(vertices);
                        for (int i = 0; i < 4; i++)
                            line(frame, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0));
                    } else {
                        //ROS_INFO("detected nothing!!!");
                    }
                    break;
                case DetectionType::RedPerson:
                    if (color_detector.detectPureObject(frame, r_box,
                                                        cv::Scalar(0, 50, 50),
                                                        cv::Scalar(10, 255, 255),
                                                        cv::Scalar(160, 50, 50),
                                                        cv::Scalar(180, 255, 255))) {
                        //ROS_INFO("detected red Person!!!");
                        object_pose.calculatePoseFromRotatedBox(r_box);
                        object_pose.publishPose();
                        cv::Point2f vertices[4];
                        r_box.points(vertices);
                        for (int i = 0; i < 4; i++)
                            line(frame, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0));
                    }
                    break;
                case DetectionType::BlackCircle:
                    if (color_detector.detectBlackCircle(frame, black_center)) {
                        object_pose.calculatePoseFromPoint(black_center);
                        object_pose.publishPose();
                        cv::circle(frame, black_center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
                    }
                    break;
                case DetectionType::Car:
                    car_dAt.beginDetection();
                    if (car_dAt.detectFrame(frame, box)) {
                        // 如果检测成功,必定会发布object_pose
                        object_pose.calculatePoseFromBox(box);
                        object_pose.publishPose();
                        cv::rectangle(frame, box, CV_RGB(255, 0, 0));
                    }
                    break;
                case DetectionType::MedicalBag:
                    // 检测最大和次大轮廓
                    is_detect_object = color_detector.detectBackgroundObject(frame, r_box_1, r_box,
                                                                             cv::Scalar(0, 50, 50),
                                                                             cv::Scalar(10, 255, 255),
                                                                             cv::Scalar(160, 50, 50),
                                                                             cv::Scalar(180, 255, 255));
                    // 计算目标物的位置
                    if (is_detect_object) {
                        // ROS_INFO("detected medical bag!!!");
                        object_pose.calculatePoseFromRotatedBox(r_box);
                        object_pose.publishPose();
                        cv::Point2f vertices[4];
                        r_box.points(vertices);
                        for (int i = 0; i < 4; i++)
                            line(frame, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0));
                    } else if (r_box_1.size.area() > 5000 &&
                               abs(r_box_1.size.area() - (frame.rows * frame.cols)) > 3000) {
                        // ROS_INFO("detected red background!!!");
                        object_pose.calculatePoseFromRotatedBox(r_box_1);
                        object_pose.publishPose();
                        cv::Point2f vertices[4];
                        r_box_1.points(vertices);
                        for (int i = 0; i < 4; i++)
                            line(frame, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0));
                    } else {
                        //ROS_INFO("detected nothing!!!");
                    }
                    break;
                case DetectionType::YellowPerson:
                    if (color_detector.detectPureObject(frame, r_box,
                                                        cv::Scalar(10, 50, 50),
                                                        cv::Scalar(30, 255, 255))) {
                        // ROS_INFO("detected yellow person!!!");
                        object_pose.calculatePoseFromRotatedBox(r_box);
                        object_pose.publishPose();
                        cv::Point2f vertices[4];
                        r_box.points(vertices);
                        for (int i = 0; i < 4; i++)
                            line(frame, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0));
                    }
                    break;
            }
            if (atoi(argv[1]) == 1) {
                // convert image from RGB to HSV
                cv::cvtColor(frame, hsv, CV_BGR2HSV);
                // show the final image
                cv::namedWindow("detection_result");
                cv::imshow("detection_result", frame);
                // click left botton of mouce then show the particular point's HSV value
                cv::setMouseCallback("detection_result", DetectionByColor::on_mouse, &hsv);
                cv::waitKey(3);
            }
        }
        rate.sleep();
    }
}

void detectionBackGroundObject() {

}

