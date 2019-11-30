#ifndef AUTO_AIMING_H
#define AUTO_AIMING_H

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/h/rs_types.h>
#include "armor_detection/armor_detection_node.h"

extern int RS2_COLOR_FRAMERATE;
extern int RS2_DEPTH_FRAMERATE;
extern int RE2_FRAME_WIDTH;
extern int RE2_FRAME_HEIGHT;
extern roborts_sdk::cmd_game_robot_state robotID;
extern int RES_HOLE_FILLING;

extern 
int auto_aiming(rs2::frameset& frames, rs2::align& align_to_color,
	rs2::hole_filling_filter& hole_filling_filter, ArmorDetectionNode& armor_detection);

int auto_aiming_init(rs2::pipeline_profile pipe_profile);
extern int remain_time;
extern int id;
#endif
