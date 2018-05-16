#include <ros/ros.h>
#include "std_msgs/Float32MultiArray.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    ImageConverter()
            : it_(nh_), box(0,0,0,0)
    {
        // Subscrive to input video feed and publish output video feed
        image_sub_ = it_.subscribe("/camera/image_raw", 1,
                                   &ImageConverter::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~ImageConverter()
    {
        cv::destroyWindow(OPENCV_WINDOW);
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

        // Draw an example circle on the video stream
        if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
            cv::rectangle(cv_ptr->image, box, CV_RGB(255,0,0));

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
    void setBox(cv::Rect2f &box)
    {
        this->box = box;
    }
private:
    cv::Rect2f box;
};

class SubObjectBox
{
public:
    SubObjectBox():box(0,0,0,0)
    {
        // Topic you want to subscribe
        sub_ = n_.subscribe("/object_boxs", 1, &SubObjectBox::callback, this);
    }

    void callback(const std_msgs::Float32MultiArray& input)
    {
        box = cv::Rect2f(input.data[1], input.data[2], input.data[3], input.data[4]);
    }

    cv::Rect2f getBox()
    {
        return box;
    }
private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    cv::Rect2f box;
};//End of class SubObjectBox

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "subscribe_and_publish");

    // Subscribe objec
    SubObjectBox sub_object_box;
    ImageConverter ic;

    while(ros::ok())
    {
        cv::Rect2f box = sub_object_box.getBox();
        ic.setBox(box);
        ros::spinOnce();
    }
    return 0;
}
