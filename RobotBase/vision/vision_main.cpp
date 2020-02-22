//
// Created by eric on 11/30/19.
//

#include "vision_main.h"

using namespace cv;

void *vision_main_function() {

    Daheng daheng;
    ArmorDetector armorDetector;
    OtherParam otherParam;
    daheng.init();
    namedWindow("control", WINDOW_AUTOSIZE);
    createTrackbar("color_th", "control", &armorDetector.color_th_, 255);
    createTrackbar("gray_th", "control", &armorDetector.gray_th_, 255);
    Mat color;
    while (1) {
        otherParam.level = gimbal->level;
        otherParam.id = gimbal->id;
        otherParam.mode = gimbal->current_mode;
        if (otherParam.id == 13) // blue 3
        {
            otherParam.color = 0; //opposite is red
        } else {
            otherParam.color = 1;
        }
        daheng.getImage(color);
//        auto time0 = static_cast<double>(getTickCount());
        armorDetector.armorTask(color,otherParam);
        //time0 = ((double) getTickCount() - time0) / getTickFrequency();
        //std::cout << "use time is " << time0 * 1000 << "ms" << std::endl;
        //imshow("lyx",color);
        if (waitKey(1) == 'q') {

            break;
        }
    }
    destroyAllWindows();
    return nullptr;
}

