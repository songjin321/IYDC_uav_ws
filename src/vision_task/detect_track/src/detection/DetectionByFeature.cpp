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
    //std::cout <<"roi size = " << roi.size << std::endl;
    //std::cout <<"roi center = " << roi.center << std::endl;
    //std::cout <<"roi angle = " << roi.angle << std::endl;
}
bool DetectionByFeature::computerH()
{

    ////////////////////////////
    // NEAREST NEIGHBOR MATCHING USING FLANN LIBRARY (included in OpenCV)
    ////////////////////////////
    cv::Mat results;
    cv::Mat dists;
    std::vector<std::vector<cv::DMatch> > matches;
    int k=2; // find the 2 nearest neighbors
    bool useBFMatcher = false; // SET TO TRUE TO USE BRUTE FORCE MATCHER
    if(objectDescriptors.type()==CV_8U)
    {
        // Binary descriptors detected (from ORB, Brief, BRISK, FREAK)
        printf("Binary descriptors detected...\n");
        if(useBFMatcher)
        {
            cv::BFMatcher matcher(cv::NORM_HAMMING); // use cv::NORM_HAMMING2 for ORB descriptor with WTA_K == 3 or 4 (see ORB constructor)
            matcher.knnMatch(objectDescriptors, sceneDescriptors, matches, k);
        }
        else
        {
            // Create Flann LSH index
            cv::flann::Index flannIndex(sceneDescriptors, cv::flann::LshIndexParams(12, 20, 2), cvflann::FLANN_DIST_HAMMING);
            // printf("Time creating FLANN LSH index = %d ms\n", time.restart());

            // search (nearest neighbor)
            flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
        }
    }
    else
    {
        // assume it is CV_32F
        printf("Float descriptors detected...\n");
        if(useBFMatcher)
        {
            cv::BFMatcher matcher(cv::NORM_L2);
            matcher.knnMatch(objectDescriptors, sceneDescriptors, matches, k);
        }
        else
        {
            // Create Flann KDTree index
            cv::flann::Index flannIndex(sceneDescriptors, cv::flann::KDTreeIndexParams(), cvflann::FLANN_DIST_EUCLIDEAN);
            //printf("Time creating FLANN KDTree index = %d ms\n", time.restart());

            // search (nearest neighbor)
            flannIndex.knnSearch(objectDescriptors, results, dists, k, cv::flann::SearchParams() );
        }
    }
    //printf("Time nearest neighbor search = %d ms\n", time.restart());

    // Conversion to CV_32F if needed
    if(dists.type() == CV_32S)
    {
        cv::Mat temp;
        dists.convertTo(temp, CV_32F);
        dists = temp;
    }

    // Find correspondences by NNDR (Nearest Neighbor Distance Ratio)
    float nndrRatio = 0.8f;
    std::vector<cv::Point2f> mpts_1, mpts_2; // Used for homography
    std::vector<int> indexes_1, indexes_2; // Used for homography
    std::vector<uchar> outlier_mask;  // Used for homography
    // Check if this descriptor matches with those of the objects
    if(!useBFMatcher)
    {
        for(int i=0; i<objectDescriptors.rows; ++i)
        {
            // Apply NNDR
            //printf("q=%d dist1=%f dist2=%f\n", i, dists.at<float>(i,0), dists.at<float>(i,1));
            if(results.at<int>(i,0) >= 0 && results.at<int>(i,1) >= 0 &&
               dists.at<float>(i,0) <= nndrRatio * dists.at<float>(i,1))
            {
                mpts_1.push_back(objectKeypoints.at(i).pt);
                indexes_1.push_back(i);

                mpts_2.push_back(sceneKeypoints.at(results.at<int>(i,0)).pt);
                indexes_2.push_back(results.at<int>(i,0));
            }
        }
    }
    else
    {
        for(unsigned int i=0; i<matches.size(); ++i)
        {
            // Apply NNDR
            //printf("q=%d dist1=%f dist2=%f\n", matches.at(i).at(0).queryIdx, matches.at(i).at(0).distance, matches.at(i).at(1).distance);
            if(matches.at(i).size() == 2 &&
               matches.at(i).at(0).distance <= nndrRatio * matches.at(i).at(1).distance)
            {
                mpts_1.push_back(objectKeypoints.at(matches.at(i).at(0).queryIdx).pt);
                indexes_1.push_back(matches.at(i).at(0).queryIdx);

                mpts_2.push_back(sceneKeypoints.at(matches.at(i).at(0).trainIdx).pt);
                indexes_2.push_back(matches.at(i).at(0).trainIdx);
            }
        }
    }
    // FIND HOMOGRAPHY
    unsigned int minInliers = 8;
    if(mpts_1.size() >= minInliers)
    {
        H = findHomography(mpts_1,
                                   mpts_2,
                                   cv::RANSAC,
                                   1.0,
                                   outlier_mask);
        int inliers=0, outliers=0;
        for(unsigned int k=0; k<mpts_1.size();++k)
        {
            if(outlier_mask.at(k))
            {
                ++inliers;
            }
            else
            {
                ++outliers;
            }
        }

        std::cerr << "corresponds point size = " << mpts_1.size()
                  << " nbMatches = " << minInliers << std::endl
                  << "inliers numbers = " << inliers << std::endl
                  << "outliers numbers = " << outliers << std::endl;

        if (!H.empty() && inliers*2.0 > outliers)
       {
           std::cerr << "computer H OK!" << std::endl;
           return true;
       }
    } else {
        printf("Not enough matches (%d) for homography...\n", (int)mpts_1.size());
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
    vertexs.clear();
    vertexs.push_back(pt1);
    vertexs.push_back(pt2);
    vertexs.push_back(pt3);
    vertexs.push_back(pt4);
    std::cout << "pt1 = " << pt1 << std::endl;
    std::cout << "pt2 = " << pt2 << std::endl;
    std::cout << "pt3 = " << pt3 << std::endl;
    std::cout << "pt4 = " << pt4 << std::endl;
}
