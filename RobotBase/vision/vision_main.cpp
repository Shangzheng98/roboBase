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
    if (daheng.init() == 0)
    {
        printf("fail to init lib\n");
        return nullptr;
    }
    namedWindow("control", WINDOW_AUTOSIZE);
    createTrackbar("color_th", "control", &armorDetector.color_th_, 255);
    createTrackbar("gray_th", "control", &armorDetector.gray_th_, 255);
    Mat color;
    BigbufDetection bigbuff =  BigbufDetection(640,480,1);
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
        resize(color,color,Size(680,460));
        imshow("a",color);
        if (color.empty())
        {
            printf("\nthe image is empty, please check camera!\n");

            break;
        }
//        auto time0 = static_cast<double>(getTickCount());
        //armorDetector.armorTask(color,otherParam);
        //time0 = ((double) getTickCount() - time0) / getTickFrequency();
        //std::cout << "use time is " << time0 * 1000 << "ms" << std::endl;
        bigbuff.feed_im(color);
        bigbuff.getTest_result();
        //imshow("lyx",color);
        if (waitKey(1) == 'q') {

            break;
        }
    }
    destroyAllWindows();
    return nullptr;
}

