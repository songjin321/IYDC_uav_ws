//
// Created by songjin on 18-6-10.
//
#include <ros/ros.h>
#include "manipulater_controller/ControlManipulater.h"
#include "MySerial.h"
#include <thread>
bool manipulater_server_callback(manipulater_controller::ControlManipulater::Request &req,
                            manipulater_controller::ControlManipulater::Response &res,
                            MySerial *my_serial)
{
    my_serial->write(req.cmd, 0);
    ros::Time init_time = ros::Time::now();
    uint8_t return_type;
    std::vector<double> return_value;
    while(ros::Time::now() < init_time + ros::Duration(8))
    {
        if (my_serial->read(return_type, return_value))
        {
            switch(return_type)
            {
                case 0:
                    ROS_INFO("is_done ok");
                    return true;
                case 1:
                    ROS_INFO("is_done failed");
                    return false;
                default:
                    break;
            }
        }
        usleep(500);
    }
    ROS_ERROR("timeout 8s");
    return false;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "manipulater_server");
    ros::NodeHandle nh;

    // port, baudrate, timeout in milliseconds
    std::string com_name;
    int baud;
    nh.param<std::string>("COM_dev",com_name,"/dev/pts/8");
    nh.param<int>("baud_rate",baud, 115200);
    MySerial my_serial(com_name, static_cast<uint32_t >(baud));
    if(!my_serial.serial.isOpen())
    {
        ROS_ERROR("The serial port does not open");
    }

    ros::ServiceServer service = nh.advertiseService<manipulater_controller::ControlManipulater::Request,
            manipulater_controller::ControlManipulater::Response>
            ("manipulater_server", boost::bind(manipulater_server_callback, _1, _2, &my_serial));

    ROS_INFO("Ready to control manipulater");

    while(ros::ok())
    {
        ros::Rate rate(100);
        ros::spinOnce();
        rate.sleep();
    }
}
