//
// Created by eric on 11/30/19.
//

#include "vision_main.h"
#include "../gimbal.h";
void *vision_main_function() {
    rs2::config cfg;
    rs2::pipeline pipe;
    rs2::pipeline_profile pipeline_Profile;
    load_camera(cfg, pipe, pipeline_Profile);

    // running with 2 mode
    while (1) {

        switch (gimbal->getCurrentMode()) {
            case 0:
                //TODO: auto-aim
                break;
            case 1:
                big_buff_init();
                big_buff();

                break;
            default:
                break;
        }
    }
}

void load_camera(rs2::config cfg, rs2::pipeline pipe, rs2::pipeline_profile profile) {
    //TODO: check the resolution
    cfg.enable_stream(RS2_STREAM_COLOR, 840, 480, RS2_FORMAT_BGR8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, 840, 480, RS2_FORMAT_Z16, 60);
    profile = pipe.start(cfg);

    rs2::align align_to_color(RS2_STREAM_COLOR);
    //rs2::hole_filling_filter hole_Filling_Filter;
    //hole_Filling_Filter.set_option(RS2_OPTION_HOLES_FILL,2);
}

