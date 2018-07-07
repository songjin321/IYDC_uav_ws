//
// Created by songjin on 18-6-10.
//
#include <ros/ros.h>
#include "manipulater_controller/ControlManipulater.h"
#include "MySerial.h"
#include <thread>
#include <mutex>
#include <MySerial.h>
CtlrStateTypeDef return_value;
std::mutex mtx;
void serialRead(MySerial *my_serial)
{
    mtx.lock();
    my_serial->read(return_value);
    mtx.unlock();
}
bool manipulater_server_callback(manipulater_controller::ControlManipulater::Request &req,
                            manipulater_controller::ControlManipulater::Response &res,
                            MySerial *my_serial)
{
    mtx.lock();
    my_serial->write(req.cmd, 0);
    mtx.unlock();
    int timeout = 7;
    bool is_begin_sing;
    bool is_begin_catch;
    bool is_begin_stretch;
    ros::Time init_time = ros::Time::now();
    while(ros::Time::now() < init_time + ros::Duration(timeout))
    {
        ROS_INFO("the state of singing: %d", return_value.status_code.singing);
        ROS_INFO("the state of motion: %d", return_value.status_code.motion);
        ROS_INFO("the state of catched: %d", return_value.status_code.catched);

        // catch completed return true
        if ( return_value.status_code.motion == 1)
        {
            is_begin_catch = true;
        }
        if (req.cmd == 3 && return_value.status_code.motion == 3 && is_begin_catch)
        {
            is_begin_catch = false;
            res.isOk = true;
            return true;
        }
        if (return_value.status_code.catched != 0)
        {
            res.isOk = false;
            return false;
        }

        // stretch completed retrun true
        if ( return_value.status_code.motion == 2)
        {
            is_begin_stretch = true;
        }
        if (req.cmd == 4 && return_value.status_code.motion == 0 && is_begin_stretch)
        {
            is_begin_stretch = false;
            res.isOk = true;
            return true;
        }

        // sing completed return true
        if ( return_value.status_code.singing == 1)
        {
            is_begin_sing = true;
        }
        if (req.cmd == 5 && return_value.status_code.singing == 0 && is_begin_sing)
        {
            is_begin_sing = false;
            res.isOk = true;
            return true;
        }

        usleep(100);
    }
    ROS_WARN("manipulater controller timeout %ds", timeout);
    res.isOk = false;
    return false;
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "manipulater_server");
    ros::NodeHandle nh;
    // port, baudrate, timeout in milliseconds
    std::string com_name;
    int baud;
    nh.param<std::string>("COM_dev",com_name,"/dev/manipulater");
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

    // create a thread to get return value;
    std::thread t_serial_read(serialRead, &my_serial);
    while(ros::ok())
    {
        ros::Rate rate(30);
        ros::spinOnce();
        rate.sleep();
    }
}
