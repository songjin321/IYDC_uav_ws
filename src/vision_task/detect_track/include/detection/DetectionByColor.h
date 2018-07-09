//
// Created by songjin on 18-5-22.
//

#ifndef UAV_WS_DETECTIONBYCOLOR_H
#define UAV_WS_DETECTIONBYCOLOR_H

#include <detection/DetectionBase.h>

class DetectionByColor: public DetectionBase
{
public:
    //　返回一个大轮廓和一个小轮廓
    //　检测到目标物体返回true,否则返回false
    bool detectBackgroundObject(cv::Mat &sceneImg, cv::RotatedRect &roi_1, cv::RotatedRect &roi_2,
                                cv::Scalar hsv_object_l1, cv::Scalar hsv_object_h1,
                                cv::Scalar hsv_object_l2 = cv::Scalar(0, 50, 50),
                                cv::Scalar hsv_object_h2 = cv::Scalar(180, 255, 255));
    bool detectPureObject(cv::Mat &sceneImg, cv::RotatedRect &roi,
                          cv::Scalar hsv_object_l1, cv::Scalar hsv_object_h1,
                          cv::Scalar hsv_object_l2 = cv::Scalar(0, 50, 50),
                          cv::Scalar hsv_object_h2 = cv::Scalar(180, 255, 255));
    bool detectBlackCircle(cv::Mat &sceneImg, cv::Point2f &center);

    // 用基于颜色的方法检测医药包
    bool detect(cv::Mat &sceneImg, cv::RotatedRect &roi) override;

    static void on_mouse(int EVENT, int x, int y, int flags, void* userdata);
};
#endif //UAV_WS_DETECTIONBYCOLOR_H
