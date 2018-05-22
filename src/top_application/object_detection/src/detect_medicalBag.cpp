#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include <limits>
#include <algorithm>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <iostream>
#include <cstring>
#include "ros/ros.h"
#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
class SubToBoxs
{
public:
    SubToBoxs()
    {
        //Topic you want to publish
        box_pub = n_.advertise<std_msgs::Float32MultiArray>("/object_boxs", 1);
        std::cerr << "BiGodSayOK" << std::endl;
        //Topic you want to subscribe
        objects_sub = n_.subscribe("/objects", 1, &SubToBoxs::objects_sub_callback, this);
    }

    void objects_sub_callback(const std_msgs::Float32MultiArray& msg)
    {
        if (msg.data.empty())
            return;
        std_msgs::Float32MultiArray box;
        // the id of the object
        box.data.push_back(msg.data[0]);

        float ow = msg.data[1];
        float oh = msg.data[2];
        float h11 = msg.data[3];
        float h12 = msg.data[4];
        float h13 = msg.data[5];
        float h21 = msg.data[6];
        float h22 = msg.data[7];
        float h23 = msg.data[8];
        float h31 = msg.data[9];
        float h32 = msg.data[10];
        float h33 = msg.data[11];

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
            return;
        ROS_INFO("x0 = %f, x1 = %f, x2 = %f, x3 = %f, y0 = %f, y1 = %f, y2 = %f, y3 = %f",
                 x0, x1, x2, x3, y0, y1, y2, y3);
        // upper left corner and lower right corner
        float xmin = std::min(x0, std::min(x1, std::min(x2, x3)));
        float xmax = std::max(x0, std::max(x1, std::max(x2, x3)));
        float ymin = std::min(y0, std::min(y1, std::min(y2, y3)));
        float ymax = std::max(y0, std::max(y1, std::max(y2, y3)));

        box.data.push_back(xmin);
        box.data.push_back(ymin);
        box.data.push_back(xmax-xmin);
        box.data.push_back(ymax-ymin);
        box_pub.publish(box);
        ROS_INFO("x = %f, y = %f, width = %f, high = %f",
                 xmin, ymin, xmax-xmin, ymax-ymin);
    }

private:
    ros::NodeHandle n_;
    ros::Publisher box_pub;
    ros::Subscriber objects_sub;
private:
    bool isRectangle(double x1, double y1,
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
        std::cout<<"mean = " << mean << " var = " << var << std::endl;
        if (25 * sqrt(var) > mean)
            return false;
        else
            return true;
    }
};//End of class SubscribeAndPublish


int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_medicalBag");
    enum State{detection, track_init, tracking};
    State state = detection;
    class RosMessageConvert a;
    SubToBoxs subToBoxs;
    DetectionByFeature detector(path_object, feature_type);
    std::shared_ptr<cv::Tracker> tracker = cv::Tracker::create("KCF");
    while(ros::ok())
    {
        cv::Mat frame = a.;
        cv::Rect2f roi;
        if( state == detection)
        {
            if(detector.detect(frame, roi))
                state = track_init;
        }
        else
        {
            if (state == track_init)
            {
                tracker->init(frame, roi);
                state = tracking;
            }
            else
            {
                tracker->update(frame, roi);
            }
        }
        ros::spinOnce();
    }
}

