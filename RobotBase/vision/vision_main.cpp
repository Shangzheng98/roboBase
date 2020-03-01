//
// Created by eric on 11/30/19.
//

#include <roboBase/RobotBase/vision/bigbuff/BigbuffDetection.h>
#include "vision_main.h"

using namespace cv;

void *vision_main_function() {

    Daheng daheng;
    ArmorDetector armorDetector;
    OtherParam otherParam;
    BigbufDetection  bigbuf = BigbufDetection(640,480,true);
    daheng.init();
    namedWindow("control", WINDOW_AUTOSIZE);
    createTrackbar("color_th", "control", &armorDetector.color_th_, 255);
    createTrackbar("gray_th", "control", &armorDetector.gray_th_, 255);
    createTrackbar("bigbuff_color_th", "control", &bigbuf.color_th_, 255);
    createTrackbar("bigbuff_gray_th", "control", &bigbuf.gray_th_, 255);

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
        if (color.empty())
        {
            printf("\nthe image is empty, please check camera!\n");

            break;
        }
//        auto time0 = static_cast<double>(getTickCount());
        //armorDetector.armorTask(color,otherParam);
        bigbuf.feed_im(color);
        bigbuf.getTest_result();
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

