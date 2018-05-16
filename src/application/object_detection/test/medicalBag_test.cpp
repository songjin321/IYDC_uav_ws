#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
class SubObjectBox;
static const std::string OPENCV_WINDOW = "Image window";

class SubImageAndDrawObjectBox
{

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    SubImageAndDrawObjectBox()
            : it_(nh_), box(0,0,0,0)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image_raw", 1,
                                   &SubImageAndDrawObjectBox::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);
        sub_ = nh_.subscribe("/object_boxs", 1, &SubImageAndDrawObjectBox::drawBoxCallback, this);
        cv::namedWindow(OPENCV_WINDOW);
    }

    ~SubImageAndDrawObjectBox()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void drawBoxCallback(const std_msgs::Float32MultiArray& input)
    {
        box = cv::Rect2f(input.data[1], input.data[2], input.data[3], input.data[4]);
        cv::rectangle(image, box, CV_RGB(255,0,0));
        cv::imshow(OPENCV_WINDOW, image);
        cv::waitKey(3);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        image = cv_ptr->image;
        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, image);
        cv::waitKey(3);
    }
private:
    ros::Subscriber sub_;
    cv::Rect2f box;
    cv::Mat image;
};

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "medicalBag_test");

    // Subscribe objec
    SubImageAndDrawObjectBox o;
    ros::spin();

    return 0;
}
