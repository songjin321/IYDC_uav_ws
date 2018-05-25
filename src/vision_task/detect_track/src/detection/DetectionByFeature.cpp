//
// Created by songjin on 18-5-22.
//

// OpenCV stuff
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/calib3d/calib3d.hpp> // for homography
#include "detection/DetectionByFeature.h"
#include <string>

DetectionByFeature::DetectionByFeature(std::string path_object):
        path_object_(path_object), detector(nullptr)
{
    // The detector can be any of (see OpenCV features2d.hpp):
    // cv::FeatureDetector * detector = new cv::DenseFeatureDetector();
    // cv::FeatureDetector * detector = new cv::FastFeatureDetector();
    // cv::FeatureDetector * detector = new cv::GFTTDetector();
    // cv::FeatureDetector * detector = new cv::MSER();
    // cv::FeatureDetector * detector = new cv::ORB();
     cv::FeatureDetector * detector = cv::xfeatures2d::SIFT::create();
    // cv::FeatureDetector * detector = new cv::StarFeatureDetector();
    // cv::FeatureDetector * detector = new cv::SURF(600.0);
    // cv::FeatureDetector * detector = new cv::BRISK();

    initObject();
}
void DetectionByFeature::initObject()
{
    cv::Mat objectImg = cv::imread(path_object_, cv::IMREAD_GRAYSCALE);
    object_width = objectImg.cols;
    object_height = objectImg.rows;
    detector->detect(objectImg, objectKeypoints);
    detector->compute(objectImg, objectKeypoints, objectDescriptors);
}
bool DetectionByFeature::detect(cv::Mat &sceneImg, cv::Rect2d &roi)
{
    detector->detect(sceneImg, sceneKeypoints);
    detector->compute(sceneImg, sceneKeypoints, sceneDescriptors);
    computerH();
    if(!computerBox())
        return false;
    getBox(roi);
    return true;
}
void DetectionByFeature::computerH()
{
    ////////////////////////////
    // NEAREST NEIGHBOR MATCHING USING FLANN LIBRARY (included in OpenCV)
    ////////////////////////////
    cv::Mat results;
    cv::Mat dists;
    int k=2; // find the 2 nearest neighbors
    if(objectDescriptors.type()==CV_8U)
    {
        // Binary descriptors detected (from ORB or Brief)

        // Create Flann LSH index
        cv::flann::Index flannIndex(sceneDescriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);

        // search (nearest neighbor)
        flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
    }
    else
    {
        // assume it is CV_32F
        // Create Flann KDTree index
        cv::flann::Index flannIndex(sceneDescriptors, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);

        // search (nearest neighbor)
        flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
    }

    // Conversion to CV_32F if needed
    if(dists.type() == CV_32S)
    {
        cv::Mat temp;
        dists.convertTo(temp, CV_32F);
        dists = temp;
    }

    ////////////////////////////
    // PROCESS NEAREST NEIGHBOR RESULTS
    ////////////////////////////
    // Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
    float nndrRatio = 0.8;
    std::vector<cv::Point2f> mpts_1, mpts_2; // Used for homography
    std::vector<int> indexes_1, indexes_2; // Used for homography
    std::vector<uchar> outlier_mask;  // Used for homography
    for(unsigned int i=0; i<objectDescriptors.rows; ++i)
    {
        // Check if this descriptor matches with those of the objects
        // Apply NNDR
        if(results.at<int>(i,0) >= 0 && results.at<int>(i,1) >= 0 && dists.at<float>(i,0) <= nndrRatio * dists.at<float>(i,1))
        {
            mpts_1.push_back(objectKeypoints.at(i).pt);
            indexes_1.push_back(i);

            mpts_2.push_back(sceneKeypoints.at(results.at<int>(i,0)).pt);
            indexes_2.push_back(results.at<int>(i,0));
        }
    }

    // FIND HOMOGRAPHY
    int nbMatches = 8;
    if(mpts_1.size() >= nbMatches)
    {
       H = findHomography(mpts_1,
                          mpts_2,
                          cv::RANSAC,
                          1.0,
                          outlier_mask);
    }
}

bool DetectionByFeature::computerBox()
{
    float ow = object_width;
    float oh = object_height;
    float h11 = H.at<float>(0,0);
    float h12 = H.at<float>(0,1);
    float h13 = H.at<float>(0,2);
    float h21 = H.at<float>(1,0);
    float h22 = H.at<float>(1,1);
    float h23 = H.at<float>(1,2);
    float h31 = H.at<float>(2,0);
    float h32 = H.at<float>(2,1);
    float h33 = H.at<float>(2,2);

    // coordinate of four vertexs
    float x0 = h31;
    float y0 = h32;
    float x1 = h11 * ow + h31;
    float y1 = h12 * ow + h32;
    float x2 = h21 * oh + h31;
    float y2 = h22 * oh + h32;
    float x3 = h11 * ow + h21 * oh + h31;
    float y3 = h12 * ow + h22 * oh + h32;
    bool is_rectangle = isRectangle(x0,y0,x1,y1,x2,y2,x3,y3);
    if(!is_rectangle)
        return false;
    // ROS_INFO("x0 = %f, x1 = %f, x2 = %f, x3 = %f, y0 = %f, y1 = %f, y2 = %f, y3 = %f",
    //        x0, x1, x2, x3, y0, y1, y2, y3);
    // upper left corner and lower right corner
    float xmin = std::min(x0, std::min(x1, std::min(x2, x3)));
    float xmax = std::max(x0, std::max(x1, std::max(x2, x3)));
    float ymin = std::min(y0, std::min(y1, std::min(y2, y3)));
    float ymax = std::max(y0, std::max(y1, std::max(y2, y3)));
    box = cv::Rect2f(xmin, xmax, xmax-xmin, ymax-ymin);
    return true;
}

void DetectionByFeature::getBox(cv::Rect2d &roi)
{
    roi = box;
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