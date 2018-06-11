//
// Created by songjin on 18-6-11.
//
#include "MySerial.h"
#include <time.h>
#include <unistd.h>
#include "iostream"

int main(int argc, char ** argv)
{
    MySerial my_serial("/dev/pts/19", 115200);
    uint8_t return_type;
    std::vector<double> return_value;
    bool is_success = true;
    clock_t this_time, last_time;
    while(true)
    {
        last_time = clock();
        my_serial.read(return_type, return_value);
        switch (return_type)
        {
            case 4:
                // TODO::让蜂鸣器响５秒///
                // sleep(5);
                ///////////////////////////
                // 如果成功返回０
                my_serial.write(0, 0);
                //　如果失败返回1
                // my_serial.write(1, 0);
                break;
            case 2:
                // TODO::抓住///
                ///////////////////////////
                // 如果成功返回０
                my_serial.write(0, 0);
                //　如果失败返回1
                // my_serial.write(1, 0);
                break;
            case 3:
                // TODO::松开///
                ///////////////////////////
                // 如果成功返回０
                my_serial.write(0, 0);
                //　如果失败返回1
                // my_serial.write(1, 0);
                break;
            default:
                break;
        }
        this_time = clock();
        usleep(50 - (double)(this_time - last_time)/CLOCKS_PER_SEC*1000);
        return_type = -1;
        // std::cout << "loop once" << std::endl;
    }
    return 0;
}