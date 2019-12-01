//
// Created by eric on 11/30/19.
//

#ifndef ROBOTBASE_VISION_MAIN_H
#define ROBOTBASE_VISION_MAIN_H

// basic lib
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/h/rs_types.h>

#include "big_buff.h"
#include "../gimbal.h"


void load_camera(rs2::config cfg, rs2::pipeline pipe, rs2::pipeline_profile profile);
void *vision_main_function();
#endif //ROBOTBASE_VISION_MAIN_H
