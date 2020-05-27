//
// Created by jsz on 5/15/20.
//

#ifndef ROBOTBASE_COMMON_H
#define ROBOTBASE_COMMON_H

#include <cstdint>
typedef enum _color {
    BLUE, RED, UNKNOWN
} Robot_color;

typedef enum _mode{
    BIGBUFF, AUTOAIM, IDLE
} mode;
struct _OtherParam {
    uint8_t color = UNKNOWN; //the self car color，0 blue，1 red
    uint8_t mode = AUTOAIM;
    uint8_t level = 0;
    uint8_t id = 0;
};
typedef _OtherParam OtherParam;
#endif //ROBOTBASE_COMMON_H
