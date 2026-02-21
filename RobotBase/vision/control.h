//
// Created by jsz on 12/27/19.
//


#ifndef ROBOTBASE_CONTROL_H
#define ROBOTBASE_CONTROL_H
#define ROI_ENABLE 1
#define SHOW_BINARY 0
#define SHOW_LIGHT_CONTOURS 1

#define FAST_DISTANCE 0
#define SHOW_FINAL_ARMOR 1
#define SHOW_ROI 1
#define SHOW_DRAW_SPOT 1
#define SHOW_LAST_TARGET 1

// PERF-1: master switch — any SHOW_* flag being 1 enables debug display (imshow/waitKey)
#define DEBUG_DISPLAY (SHOW_BINARY || SHOW_LIGHT_CONTOURS || SHOW_FINAL_ARMOR || SHOW_ROI || SHOW_DRAW_SPOT || SHOW_LAST_TARGET)


#endif //ROBOTBASE_CONTROL_H
