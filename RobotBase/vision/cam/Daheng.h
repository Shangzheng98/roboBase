//
// Created by jsz on 2/15/20.
//

#ifndef ROBOTBASE_DAHENG_H
#define ROBOTBASE_DAHENG_H

#include <opencv2/opencv.hpp>
#include "../../GxIAPI.h"
#include "../../DxImageProc.h"

using namespace cv;
using namespace std;

class Daheng {
public:
    ~Daheng();
    int init();
    void getImage(Mat &img);
    uint64_t getFrameNumber();
    Daheng();

private:
    GX_STATUS status;
    GX_DEV_HANDLE hDevice;
    GX_OPEN_PARAM stOpenParam;
    uint32_t nDeviceNum;
    GX_FRAME_DATA stFrameData;
    Mat src;
    uint64_t nFrameNum;
};


#endif //ROBOTBASE_DAHENG_H
