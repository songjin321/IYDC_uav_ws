
#include <opencv2/opencv.hpp>
#include "detection/DetectionByColor.h"
using namespace cv;
bool isSmaller(const std::vector<cv::Point> &s1, const std::vector<cv::Point> &s2)
{
    return s1.size() < s2.size();
}

int main(int argc, char** argv)
{
    // 提取绿色为感兴趣区域
    // green is 1, others is 0
    cv::Mat srcImage = cv::imread("/home/songjin/Project/uav_ws/exp_data/1.png");
    cv::RotatedRect r_box;
    cv::Mat hsvImg;
    cv::cvtColor(srcImage, hsvImg, CV_BGR2HSV);
    cv::Scalar hsv_l(40,50,50);
    cv::Scalar hsv_h(80,255,255);
    cv::Mat bw;
    inRange(hsvImg, hsv_l, hsv_h, bw);
    imshow("Specific Colour", bw);
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

    // 找到同心圆的原点
    cv::Mat imCrop = srcImage(r_box.boundingRect());
    cv::Mat src_gray;
    /// Convert it to gray
    cvtColor( imCrop, src_gray, COLOR_BGR2GRAY );

    /// Reduce the noise so we avoid false circle detection
    GaussianBlur( src_gray, src_gray, Size(9, 9), 2, 2 );

    std::vector<Vec3f> circles;

    /// Apply the Hough Transform to find the circles
    HoughCircles( src_gray, circles, HOUGH_GRADIENT, 1, src_gray.rows/8, 200, 100, 0, 0 );
    std::cout << "circle number: " << circles.size() <<std::endl;
    /// Draw the circles detected
    Point circle_center_sum(0,0);
    for( size_t i = 0; i < circles.size(); i++ )
    {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        circle( imCrop, center, 3, Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        circle( imCrop, center, radius, Scalar(0,0,255), 3, 8, 0 );
        circle_center_sum = circle_center_sum + center;

    }

    //Point2f circle_center((circle_center_sum.x + r_box.boundingRect().x)/circles.size(),
    //                      circle_center_sum.y + r_box.boundingRect().y/circles.size());
    /// Show your results
    namedWindow( "Hough Circle Transform Demo", WINDOW_AUTOSIZE );
    imshow( "Hough Circle Transform Demo", imCrop );

    // circle center
    //circle( srcImage, circle_center, 3, Scalar(0,255,0), -1, 8, 0 );
    namedWindow( "Circle center point", WINDOW_AUTOSIZE );
    imshow( "Circle center point", srcImage );

    waitKey(0);
    return 0;

}