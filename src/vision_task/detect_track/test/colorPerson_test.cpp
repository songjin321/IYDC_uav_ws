//
// Created by songjin on 18-5-22.
//
#include "detection/DetectionByColor.h"
#include <opencv2/opencv.hpp>

bool isSmaller(const std::vector<cv::Point> &s1, const std::vector<cv::Point> &s2)
{
    return s1.size() < s2.size();
}

int main(int argc, char **argv)
{
    /*
// green is 1, others is 0
//DetectionByColor colorPerson(40, 80);
cv::Mat srcImage = cv::imread("/home/songjin/Project/uav_ws/exp_data/red_color_person.png");
cv::RotatedRect r_box;
cv::Mat hsvImg;
cv::cvtColor(srcImage, hsvImg, CV_BGR2HSV);
cv::Mat mask1, mask2;
inRange(hsvImg, cv::Scalar(0, 50, 50), cv::Scalar(10, 255, 255), mask1);
inRange(hsvImg, cv::Scalar(170, 50, 50), cv::Scalar(180, 255, 255), mask2);
cv::Mat bw = mask1 | mask2;

std::vector< std::vector<cv::Point> > contours;
std::vector<cv::Vec4i> hierarchy;
cv::findContours(bw, contours, hierarchy, CV_RETR_LIST, cv::CHAIN_APPROX_NONE, cv::Point());
std::stable_sort(contours.begin(), contours.end(), isSmaller);
//
r_box = cv::minAreaRect(*(contours.end()-1));

cv::Point2f vertices[4];
r_box.points(vertices);
for (int i = 0; i < 4; i++)
    line(srcImage, vertices[i], vertices[(i+1)%4], cv::Scalar(0,255,0));
cv::imshow("test_result", srcImage);
cv::waitKey(0);
return 0;
 */
    system("roslaunch competition_tasks find_object.launch &");
    std::cout << "begin" << std::endl;
    std::string str = "";
    char ch;
    while ((ch = std::cin.get()) != 27) {
        std::cout << "ha ah ah" << std::endl;
    }
    system("pkill find_object_2d");
    std::cout << "exit" << std::endl;
}