#ifndef BIGBUFF_H
#define BIGBUFF_H

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/h/rs_types.h>


extern int DEBUG_ENABLE;
extern int ENEMY_COLOR;

extern int redLowH1;
extern int redHighH1;
extern int redLowH2;
extern int redHighH2;

extern int blueLowH;
extern int blueHighH;

extern int LowS;
extern int HighS;

extern int LowV;
extern int HighV;

extern int dilation_size;
extern int erosion_size;

extern int center_size_min;
extern int center_size_max;

extern int circle_radius_min;
extern int circle_radius_max;

extern int estimate_angle;
extern int estimate_height;

extern int remain_time;
extern int id;
extern int direction;
#define BLUE 0
#define RED 1

extern int RE2_FRAME_WIDTH;
extern int RE2_FRAME_HEIGHT;

int big_buff_init(rs2::pipeline_profile pipe_profile);
int big_buff(rs2::frameset frames);

#endif
