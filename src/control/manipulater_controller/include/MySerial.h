//
// Created by songjin on 18-6-10.
//

#ifndef MANIPULATER_CONTROLLER_MYSERIAL_H
#define MANIPULATER_CONTROLLER_MYSERIAL_H
#include "CRC.h"
#include "serial/serial.h"
#include <stdio.h>
class MySerial
{
public:
    MySerial(std::string com_name, uint32_t baud):
            serial(com_name, baud, serial::Timeout::simpleTimeout(100))
    {

    }
    //0x04  让蜂鸣器响５秒
    //0x02  执行机构抓住
    //0x03  执行机构松开
    //0x00  执行成功
    //0x01  执行失败
    void write(uint8_t msg_type, double value)
    {
        uint8_t length = 14;
        uint8_t buff[length];
        ((uint16_t *)buff)[0] = 0x85AA;
        memcpy(&buff[2], &length, 1);
        memcpy(&buff[3], &msg_type, 1);
        memcpy(&buff[4], &value, 8);
        uint16_t crc = CRC::Calculate(buff, 12, CRC::CRC_16_BUYPASS());
        memcpy(&buff[12], &crc, 2);
        size_t bytes_wrote = serial.write(buff, length);
    }
    bool read(uint8_t &msg_type, std::vector<double> &value)
    {
        uint8_t length;
        uint8_t header[2] = {0x00, 0x00};
        size_t bytes_read = serial.read(header, 2);
        //printf(" %x", header[0]);
        //printf(" %x \n", header[1]);
        if (((uint16_t *)header)[0] != 0x85AA)
        {
            printf(" header not equal\n");
            return false;
        }

        serial.read(&length, 1);
        uint8_t read_data[length];
        ((uint16_t *)read_data)[0] = 0x85AA;
        read_data[2] = length;
        serial.read(read_data+3, length-3);
        uint16_t crc = CRC::Calculate(read_data, length-2, CRC::CRC_16_BUYPASS());
        uint16_t crc_get = ((uint16_t *)(&read_data[length-2]))[0];
        if (crc != crc_get)
        {
            printf(" CRC not equal, CRC calculated is %x, get is %x\n", crc, crc_get);
            return false;
        }
        msg_type = read_data[3];
        printf(" msg_type = %d", msg_type);
        for (int i = 4; i + 8 < length; i+=8)
        {
            value.push_back(*((float *)(&read_data[i])));
        }
    }
    serial::Serial serial;
    bool is_done=false;
};
#endif //MANIPULATER_CONTROLLER_MYSERIAL_H
