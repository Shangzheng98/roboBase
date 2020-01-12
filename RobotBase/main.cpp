#include <librealsense2/rs.hpp>
#include <signal.h>
#include "vision/vision_main.h"
#include "roborts_sdk/sdk.h"
#include "vision/gimbal/gimbal.h"
#include "iniparser/iniparser.h"


// global variable
//char SERIAL_PORT[20] = "/dev/ttyACM0";
char SERIAL_PORT[20] = "/dev/serial_sdk";
#define CONNECT_TO_SERIAL 1
Gimbal *gimbal = nullptr;
std::shared_ptr<roborts_sdk::Handle> handle = nullptr;

using namespace cv;
int main(int argc, char *argv[]) {
    // Start SDK
    //start_sdk();

    // Gimbal initializin
#if CONNECT_TO_SERIAL
    auto handle = std::make_shared<roborts_sdk::Handle>(SERIAL_PORT);
    if (!handle->Init()) {
        return 1;
    }
    gimbal = new Gimbal(handle);
    //chassis = new Chassis(handle);
    namedWindow("Offset",WINDOW_AUTOSIZE);
    cv::createTrackbar("OFFSET_INT_X", "Offset", &gimbal->OFFSET_INT_X, 10000);
    cv::createTrackbar("OFFSET_INT_Y", "Offset", &gimbal->OFFSET_INT_Y, 10000);
    cv::createTrackbar("OFFSET_INT_Z", "Offset", &gimbal->OFFSET_INT_Z, 10000);
#else
    gimbal = new Gimbal(NULL);
#endif

    // New thread for vision function
    std::thread vision_main_thread = std::thread(vision_main_function);

    //Keep accepting the message from development board, the main thread services SDk
    while (1) {
        if (CONNECT_TO_SERIAL)
            handle->Spin();
        usleep(1000);

    }
    return 0;
}


