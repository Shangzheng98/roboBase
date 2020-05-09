//
// Created by jsz on 12/27/19.
//


#ifndef ROBOTBASE_CONTROL_H
#define ROBOTBASE_CONTROL_H
#define ROI_ENABLE 1
#define SHOW_BINART 0
#define SHOW_LIGHT_CONTOURS 1

#define FAST_DISTANCE 0
#define SHOW_FINAL_ARMOR 0
#define SHOW_ROI 1
#define SHOW_DRAW_SPOT 0
#define SHOW_LAST_TARGET 0

typedef enum color {
    BLUE, RED, UNKNOWN
} Robot_color;

typedef enum mode{
    BIGBUFF, AUTOAIM, IDLE
};


#endif //ROBOTBASE_CONTROL_H
