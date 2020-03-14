//
// Created by eric on 11/30/19.
//

#include "vision_main.h"
#include <roboBase/RobotBase/Robogrinder_SDK/serial_port.h>
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
    namedWindow("offset", WINDOW_AUTOSIZE);
    char name[20] = "/dev/ttyUSB0";
    serial_port sp(name, B115200);
    BigbufDetection bigbuff =  BigbufDetection(640,480);
    //createTrackbar("offset_YAW", "offset", &armorDetector.OFFSET_INT_YAW, 3600);
    //createTrackbar("offset_PITCH", "offset", &armorDetector.OFFSET_INT_PITCH, 3600);
    Mat color;

    while (1) {
        otherParam.level = 0;//gimbal->level
        otherParam.id =0;// gimbal->id;
        otherParam.mode =0;// gimbal->current_mode;

        if (otherParam.id == 13) // blue 3
        {
            otherParam.color = 0; //opposite is red
        } else {
            otherParam.color = 1;
        }
        daheng.getImage(color);
        //resize(color,color,Size(680,460));
        //imshow("a",color);
        if (color.empty())
        {
            printf("\nthe image is empty, please check camera!\n");

            break;
        }
//        auto time0 = static_cast<double>(getTickCount());
//        armorDetector.armorTask(color,otherParam,sp);
//        time0 = ((double) getTickCount() - time0) / getTickFrequency();
//        std::cout << "use time is " << time0 * 1000 << "ms" << std::endl;
        bigbuff.feed_im(color);
        bigbuff.getTest_result();
        bigbuff.make_prediction(sp);
//        imshow("test",color);
        if (waitKey(1) == 'q') {

            break;
        }
    }
    destroyAllWindows();
    return nullptr;
}

