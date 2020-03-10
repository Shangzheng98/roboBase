//
// Created by jsz on 2/15/20.
//

#include "Daheng.h"

Daheng::Daheng() {
    this->status = GX_STATUS_SUCCESS;
    this->src.create(Size(640, 480), CV_8UC3);
    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_INDEX;
    stOpenParam.pszContent = "1";
    this->nFrameNum = 0;

}

Daheng::~Daheng() {
    this->status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);
    free(stFrameData.pImgBuf);

    this->status = GXCloseDevice(hDevice);
    this->status = GXCloseLib();
}

int Daheng::init() {
    this->status = GXInitLib();
    if (this->status != GX_STATUS_SUCCESS) {
        printf("failed to initialize!!!!!!!!!!!!!!!\n");
        return 0;
    }

    this->status = GXUpdateDeviceList(&nDeviceNum, 1000);

    if (status != GX_STATUS_SUCCESS || nDeviceNum == NULL) {
        printf("cannot find the device!!!!!!!!!!\n");
    }
    status = GXOpenDevice(&stOpenParam, &hDevice);
    //std::cout << status << "success" << std::endl;
    if (status == GX_STATUS_SUCCESS) {
        int64_t nPayLoadSize = 0;
        //获取图像buffer大小，下面动态申请内存
        status = GXGetInt(hDevice, GX_INT_PAYLOAD_SIZE, &nPayLoadSize);

        if (status == GX_STATUS_SUCCESS && nPayLoadSize > 0) {
            //定义GXGetImage的传入参数

            //根据获取的图像buffer大小m_nPayLoadSize申请buffer
            stFrameData.pImgBuf = malloc((size_t) nPayLoadSize);

            // 设置曝光值
            status = GXSetFloat(hDevice, GX_FLOAT_EXPOSURE_TIME, 1500);

            //设置采集模式连续采集
            //            status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
            //            status = GXSetInt(hDevice, GX_INT_ACQUISITION_SPEED_LEVEL, 1);
            //status = GXSetEnum(hDevice, GX_ENUM_BALANCE_WHITE_AUTO, GX_BALANCE_WHITE_AUTO_CONTINUOUS);
            //status = GXSetBool(hDevice,GX_ENUM_GAIN_AUTO, false);

            //发送开始采集命令
            status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);
            return 1;
        }
    }
    return 0;

}

void Daheng::getImage(Mat &img) {
    GXFlushQueue(hDevice);
    GXGetImage(hDevice, &stFrameData, 100);
    //usleep(1);

    if (stFrameData.nStatus == GX_FRAME_STATUS_SUCCESS) {
        //图像获取成功
        char *m_rgb_image = nullptr; //增加的内容
        m_rgb_image = new char[stFrameData.nWidth * stFrameData.nHeight * 3];
        src.create(stFrameData.nHeight, stFrameData.nWidth, CV_8UC3);
        DxRaw8toRGB24(stFrameData.pImgBuf,
                      m_rgb_image, stFrameData.nWidth, stFrameData.nHeight, RAW2RGB_NEIGHBOUR3,
                      DX_PIXEL_COLOR_FILTER(BAYERBG), false);
        memcpy(src.data, m_rgb_image, stFrameData.nWidth * stFrameData.nHeight * 3);

        src.copyTo(img);
//        img = src;

        nFrameNum++;

        //对图像进行处理...
        delete[]m_rgb_image;
        //                        }
    }
}

uint64_t Daheng::getFrameNumber() {
    return nFrameNum;
}