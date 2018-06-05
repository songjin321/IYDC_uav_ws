//
// Created by songjin on 18-5-22.
//

// OpenCV stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for homography
#include "detection/DetectionByFeature.h"
#include <string>
#include <iostream>

DetectionByFeature::DetectionByFeature(std::string path_object):
        path_object_(path_object)
{
    // The detector can be any of (see OpenCV features2d.hpp):
    // cv::FeatureDetector * detector = new cv::DenseFeatureDetector();
    // cv::FeatureDetector * detector = new cv::FastFeatureDetector();
    // cv::FeatureDetector * detector = new cv::GFTTDetector();
    // cv::FeatureDetector * detector = new cv::MSER();
    detector = cv::xfeatures2d::SIFT::create();
    // cv::FeatureDetector * detector = new cv::StarFeatureDetector();
    // cv::FeatureDetector * detector = new cv::SURF(600.0);
    // cv::FeatureDetector * detector = new cv::BRISK();
    initObject();
}
void DetectionByFeature::initObject()
{
    cv::Mat objectImg = cv::imread(path_object_, cv::IMREAD_GRAYSCALE);
    if (objectImg.empty())
    {
        std::cerr << "read object template image failed" << std::endl;
        return;
    }
    object_width = objectImg.cols;
    object_height = objectImg.rows;
    detector->detect(objectImg, objectKeypoints);
    std::cout << "object template feature number: " << objectKeypoints.size() << std::endl;
    detector->compute(objectImg, objectKeypoints, objectDescriptors);
}
bool DetectionByFeature::detect(cv::Mat &sceneImg, cv::Rect2d &roi)
{
    detector->detect(sceneImg, sceneKeypoints);
    detector->compute(sceneImg, sceneKeypoints, sceneDescriptors);
    if(!computerH())
        return false;
    if(!computerBox())
        return false;
    std::cout << "box area equal to " << box.area() <<std::endl;
    if (box.area() > 100000)
        return false;
    roi = box;
    return true;
}
bool DetectionByFeature::detect(cv::Mat &sceneImg, cv::RotatedRect &roi)
{
    detector->detect(sceneImg, sceneKeypoints);
    detector->compute(sceneImg, sceneKeypoints, sceneDescriptors);
    if(!computerH())
        return false;
    computerFourVertex();
    roi = cv::minAreaRect(vertexs);
}
bool DetectionByFeature::computerH()
{
    ////////////////////////////
    // NEAREST NEIGHBOR MATCHING USING FLANN LIBRARY (included in OpenCV)
    //-- Step 3: Matching descriptor vectors using FLANN matcher
    cv::FlannBasedMatcher matcher;
    std::vector< cv::DMatch > matches;
    matcher.match(objectDescriptors, sceneDescriptors, matches);

    double max_dist = 0; double min_dist = 10000;
    //-- Quick calculation of max and min distances between keypoints
    for( int i = 0; i < objectDescriptors.rows; i++ )
    {
        double dist = matches[i].distance;
        if( dist < min_dist ) min_dist = dist;
        if( dist > max_dist ) max_dist = dist;
    }

    printf("-- Max dist : %f \n", max_dist );
    printf("-- Min dist : %f \n", min_dist );

    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
    //-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
    //-- small)
    //-- PS.- radiusMatch can also be used here.
    std::vector< cv::DMatch > good_matches;

    for( int i = 0; i < objectDescriptors.rows; i++ )
    {
        if( matches[i].distance <= std::max(min_dist+(max_dist-min_dist)/2, 0.02) )
        {
            good_matches.push_back( matches[i]);
        }
    }

    std::vector<cv::Point2f> mpts_1, mpts_2; // Used for homography
    std::vector<uchar> outlier_mask;  // Used for homography
    for( int i = 0; i < (int)good_matches.size(); i++ )
    {
        mpts_1.push_back(objectKeypoints.at(good_matches[i].queryIdx).pt);
        mpts_2.push_back(sceneKeypoints.at(good_matches[i].trainIdx).pt);
    }

    // FIND HOMOGRAPHY
    // TODO::让nbMatch作为参数方便调试
    int nbMatches = 20;
    std::cerr << "corresponds point size = " << mpts_1.size()
              << " nbMatches = " << nbMatches << std::endl;
    if(mpts_1.size() >= nbMatches)
    {
       H = findHomography(mpts_1,
                          mpts_2,
                          cv::RANSAC,
                          1.0,
                          outlier_mask);
       return true;
    } else {
        return false;
    }
}

bool DetectionByFeature::computerBox()
{
    double ow = object_width;
    double oh = object_height;
    double h11 = H.at<double>(0,0);
    double h12 = H.at<double>(0,1);
    double h13 = H.at<double>(0,2);
    double h21 = H.at<double>(1,0);
    double h22 = H.at<double>(1,1);
    double h23 = H.at<double>(1,2);
    double h31 = H.at<double>(2,0);
    double h32 = H.at<double>(2,1);
    double h33 = H.at<double>(2,2);
    // std::cout << h11 << " " << h12 << " "<< h13 << " "<< h21 << " "<< h22 << " "<< h23 << " "<< h31 << " "
    //       << h32 << " "<< h33 << std::endl;
    // coordinate of four vertexs
    double x0 = h13;
    double y0 = h23;
    double x1 = h11 * ow + h13;
    double y1 = h21 * ow + h23;
    double x2 = h12 * oh + h13;
    double y2 = h22 * oh + h23;
    double x3 = h11 * ow + h12 * oh + h13;
    double y3 = h21 * ow + h22 * oh + h23;
    bool is_rectangle = isRectangle(x0,y0,x1,y1,x2,y2,x3,y3);
    if(!is_rectangle)
        return false;
    // std::cout << "x0 = "<< x0 << " x1 = " <<  x1 << " x2 = " <<  x2
    //          << " x3 = " << x3 << " y0 = " << y0 << " y1 = " << y1 << " y2 = " << y2
    //          << " y3 = " << y3 << std::endl;
    // upper left corner and lower right corner
    double xmin = std::min(x0, std::min(x1, std::min(x2, x3)));
    double xmax = std::max(x0, std::max(x1, std::max(x2, x3)));
    double ymin = std::min(y0, std::min(y1, std::min(y2, y3)));
    double ymax = std::max(y0, std::max(y1, std::max(y2, y3)));
    box = cv::Rect2d(xmin, ymin, xmax-xmin, ymax-ymin);
    return true;
}

bool DetectionByFeature::isRectangle(double x1, double y1,
                 double x2, double y2,
                 double x3, double y3,
                 double x4, double y4)
{
    double cx,cy;
    double dd1,dd2,dd3,dd4;

    cx=(x1+x2+x3+x4)/4;
    cy=(y1+y2+y3+y4)/4;

    dd1=sqrt((cx-x1)*(cx-x1)+(cy-y1)*(cy-y1));
    dd2=sqrt((cx-x2)*(cx-x2)+(cy-y2)*(cy-y2));
    dd3=sqrt((cx-x3)*(cx-x3)+(cy-y3)*(cy-y3));
    dd4=sqrt((cx-x4)*(cx-x4)+(cy-y4)*(cy-y4));
    double mean = (dd1 + dd2 + dd3 + dd4)/4;
    double var = ((dd1-mean)*(dd1-mean) + (dd2-mean)*(dd2-mean)
                  +(dd3-mean)*(dd3-mean) + (dd4-mean)*(dd4-mean))/4;
    //std::cout<<"mean = " << mean << " var = " << var << std::endl;
    if (25 * sqrt(var) > mean)
        return false;
    else
        return true;
}

void DetectionByFeature::computerFourVertex()
{
    double ow = object_width;
    double oh = object_height;
    double h11 = H.at<double>(0,0);
    double h12 = H.at<double>(0,1);
    double h13 = H.at<double>(0,2);
    double h21 = H.at<double>(1,0);
    double h22 = H.at<double>(1,1);
    double h23 = H.at<double>(1,2);
    double h31 = H.at<double>(2,0);
    double h32 = H.at<double>(2,1);
    double h33 = H.at<double>(2,2);
    // std::cout << h11 << " " << h12 << " "<< h13 << " "<< h21 << " "<< h22 << " "<< h23 << " "<< h31 << " "
    //       << h32 << " "<< h33 << std::endl;
    // coordinate of four vertexs
    cv::Point pt1, pt2, pt3, pt4;
    pt1.x = static_cast<int>(h13);
    pt1.y = static_cast<int>(h23);
    pt2.x = static_cast<int>(h11 * ow + h13);
    pt2.y = static_cast<int>(h21 * ow + h23);
    pt3.x = static_cast<int>(h12 * oh + h13);
    pt3.y = static_cast<int>(h22 * oh + h23);
    pt4.x = static_cast<int>(h11 * ow + h12 * oh + h13);
    pt4.y = static_cast<int>(h21 * ow + h22 * oh + h23);
    vertexs.push_back(pt1);
    vertexs.push_back(pt2);
    vertexs.push_back(pt3);
    vertexs.push_back(pt4);
}
