//
// Created by songjin on 18-5-22.
//
#include "detection/DetectionByColor.h"
#include <opencv2/opencv.hpp>
using namespace cv;
bool isSmaller(const std::vector<cv::Point> &s1, const std::vector<cv::Point> &s2)
{
    return s1.size() < s2.size();
}

bool DetectionByColor::detectBackgroundObject(cv::Mat &sceneImg, cv::RotatedRect &roi_1, cv::RotatedRect &roi_2,
                                              cv::Scalar hsv_background_l, cv::Scalar hsv_background_h) {
    // 将RGB转化为HSV
    cv::Mat hsvImg;
    cv::cvtColor(sceneImg, hsvImg, CV_BGR2HSV);


    // HSV 阈值分割
    cv::Mat bw;
    inRange(hsvImg, hsv_background_l, hsv_background_h, bw);

    // 进行腐蚀消除一部分噪点
    cv::Mat erodeImg;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(bw, erodeImg, element);


    // 进行膨胀变回原来的形状
    cv::Mat dilateImg;
    cv::erode(erodeImg, dilateImg, element);


    // 查找轮廓
    std::vector <std::vector<cv::Point>> contours;
    std::vector <cv::Vec4i> hierarchy;
    cv::findContours(dilateImg, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point());

    // 查找轮廓当中最大的填充面积
    double largest_area_1 = -1;
    int largest_contour_index_1 = -1;
    double largest_area_2 = -1;
    int largest_contour_index_2 = -1;

    for (size_t i = 0; i < contours.size(); i++)  // 遍历每个轮廓
    {
        double area = cv::contourArea(contours[i]);  // 计算每个轮廓的面积

        if (area > largest_area_1) {
            largest_area_1 = area;
            largest_contour_index_1 = i;               // 储存最大轮廓的数字
            largest_area_2 = largest_area_1;
            largest_contour_index_2 = largest_contour_index_1;
        }else if(area > largest_area_2){
            largest_area_2 = area;
            largest_contour_index_2 = i;
        }
    }
    // 如果最大区域面积小于4000，证明视野范围之内没有板子
    if (largest_area_1 < 4000) {
        return false;
    }// 如果最大区域面积大于4000，证明视野进入绿色板子以内
    else {
        //最大面积的最小外接矩形
        roi_1 = cv::minAreaRect(contours[largest_contour_index_1]);

        // 只有一个轮廓时不对第二个轮廓赋值
        if(contours.size() == 1)
            return false;

        //第二大面积的最小外接矩形
        roi_2 = cv::minAreaRect(contours[largest_contour_index_2]);

        //求中心像素的 3x3 mask 的 HSV 平均值
        cv::Point2f center = roi_2.center;

        int h_avg;
        int s_avg;
        int v_avg;

        h_avg = (int) ((hsvImg.at<Vec3b>(center.y - 1, center.x - 1)[0] + hsvImg.at<Vec3b>(center.y, center.x - 1)[0] +
                        hsvImg.at<Vec3b>(center.y + 1, center.x - 1)[0]
                        + hsvImg.at<Vec3b>(center.y - 1, center.x)[0] + hsvImg.at<Vec3b>(center.y, center.x)[0] +
                        hsvImg.at<Vec3b>(center.y + 1, center.x)[0]
                        + hsvImg.at<Vec3b>(center.y - 1, center.x + 1)[0] +
                        hsvImg.at<Vec3b>(center.y, center.x + 1)[0] + hsvImg.at<Vec3b>(center.y + 1, center.x + 1)[0]) /
                       9);

        s_avg = (int) ((hsvImg.at<Vec3b>(center.y - 1, center.x - 1)[1] + hsvImg.at<Vec3b>(center.y, center.x - 1)[1] +
                        hsvImg.at<Vec3b>(center.y + 1, center.x - 1)[1]
                        + hsvImg.at<Vec3b>(center.y - 1, center.x)[1] + hsvImg.at<Vec3b>(center.y, center.x)[1] +
                        hsvImg.at<Vec3b>(center.y + 1, center.x)[1]
                        + hsvImg.at<Vec3b>(center.y - 1, center.x + 1)[1] +
                        hsvImg.at<Vec3b>(center.y, center.x + 1)[1] + hsvImg.at<Vec3b>(center.y + 1, center.x + 1)[1]) /
                       9);

        v_avg = (int) ((hsvImg.at<Vec3b>(center.y - 1, center.x - 1)[2] + hsvImg.at<Vec3b>(center.y, center.x - 1)[2] +
                        hsvImg.at<Vec3b>(center.y + 1, center.x - 1)[2]
                        + hsvImg.at<Vec3b>(center.y - 1, center.x)[2] + hsvImg.at<Vec3b>(center.y, center.x)[2] +
                        hsvImg.at<Vec3b>(center.y + 1, center.x)[2]
                        + hsvImg.at<Vec3b>(center.y - 1, center.x + 1)[2] +
                        hsvImg.at<Vec3b>(center.y, center.x + 1)[2] + hsvImg.at<Vec3b>(center.y + 1, center.x + 1)[2]) /
                       9);

        // 判断 h_avg 范围是否在 [95,130] 之间
        if (h_avg > 95 && h_avg < 130) {
            return true;  //中心是蓝色
        } else {
            return false; //中心不是蓝色
        }
    }
}

bool DetectionByColor::detectBlackCircle(cv::Mat &sceneImg, cv::Point2f &center)
{
    cv::RotatedRect r_box, r_box2;
    detectBackgroundObject(sceneImg, r_box, r_box2, cv::Scalar(40,0,0), cv::Scalar(80,255,255));
    cv::Mat imCrop = sceneImg(r_box.boundingRect());
    cv::Mat src_gray;
    /// Convert it to gray
    cvtColor( imCrop, src_gray, cv::COLOR_BGR2GRAY );

    /// Reduce the noise so we avoid false circle detection
    GaussianBlur( src_gray, src_gray, cv::Size(9, 9), 2, 2 );

    std::vector<cv::Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    HoughCircles( src_gray, circles, cv::HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
    std::cout << "circle number: " << circles.size() <<std::endl;

    if (circles.empty())
        return false;

    /// calculate the average center
    cv::Point2f circle_center_sum(0.0, 0.0);
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point2f center_(cvRound(circles[i][0]), cvRound(circles[i][1]));
        circle_center_sum = circle_center_sum + center_;
    }

    center.x = circle_center_sum.x/circles.size() + r_box.boundingRect().x,
    center.y = circle_center_sum.y/circles.size() + r_box.boundingRect().y;
    return true;
}

bool DetectionByColor::detectPureObject(cv::Mat &sceneImg, cv::RotatedRect &roi,
                                        cv::Scalar hsv_object_l1, cv::Scalar hsv_object_h1,
                                        cv::Scalar hsv_object_l2, cv::Scalar hsv_object_h2)
{
    cv::Mat hsvImg;
    cv::cvtColor(sceneImg, hsvImg, CV_BGR2HSV);
    cv::Mat mask1, mask2;
    inRange(hsvImg, hsv_object_l1, hsv_object_h1, mask1);
    inRange(hsvImg, hsv_object_l2, hsv_object_h2, mask2);
    cv::Mat bw = mask1 | mask2;

    std::vector< std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bw, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point());
    std::stable_sort(contours.begin(), contours.end(), isSmaller);
    //
    roi = cv::minAreaRect(*(contours.end()-1));
}
