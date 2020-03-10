//
// Created by jsz on 3/8/20.
//

#ifndef SERIAL_PORT_MESSAGE_H
#define SERIAL_PORT_MESSAGE_H

#include <stdint.h>
struct serial_gimbal_data
{
    uint8_t rowData[20];
    int head = 0xaf;
    int id = 1;
    int size;
};

struct serial_friction_data
{
    char rowData[20];
    int head = 0xcc;
    int id  = 2;
    int size;
};

struct serical_recive_data
{
    char rowData[10];
    int head = 0xaa;
    int id = 3;
    int size;
};

struct gimbal_msg
{
    int16_t pitch;
    int16_t yaw;
};

struct friction_msg
{
    uint16_t left;
    uint16_t right;

};

struct recive_msg
{
    uint8_t robot_id;
    uint8_t rebot_level;
    uint8_t vison_mode;
};
#endif //SERIAL_PORT_MESSAGE_H
