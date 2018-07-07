//
// Created by songjin on 18-6-10.
//

#ifndef MANIPULATER_CONTROLLER_MYSERIAL_H
#define MANIPULATER_CONTROLLER_MYSERIAL_H
#include "CRC.h"
#include "serial/serial.h"
#include <stdio.h>
#define CTLR_STAT_DEF_LENGTH 20
typedef enum
{
    CATCHED_CATCHING			=0x00,
    CATCHED_LOAD				=0x01,
    CATCHED_EMPTY				=0x02,
    CATCHED_OVERLOAD			=0x03,
} CatchedTypeDef;
typedef union
{
    struct
    {
        uint8_t singing:1;
        uint8_t motion:2; //0:released; 1:catching; 2:stretching; 3: still
        CatchedTypeDef catched:2; //when catching: 0:catching&&empty; 1:catched&&load; 2:catched&&empty
    };
    uint16_t val;
} StatusCodeTypeDef;
typedef union
{
    struct
    {
        uint32_t type0head;
        int32_t position; //angle in 0.01 degree
        float velocity;
        float load;
        StatusCodeTypeDef status_code;
        uint16_t  crc;
    };
    uint8_t buf[CTLR_STAT_DEF_LENGTH]; //!< Union --> Byte<0-7>
} CtlrStateTypeDef;

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
    bool read(CtlrStateTypeDef &return_value)
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
        memcpy(return_value.buf, read_data, length);
        // printf("recevice return value");
	return true;
    }
    serial::Serial serial;
    bool is_done=false;
};
#endif //MANIPULATER_CONTROLLER_MYSERIAL_H
