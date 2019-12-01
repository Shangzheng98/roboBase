#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <signal.h>
#include "vision/vision_main.h"
#include "roborts_sdk/sdk.h"
#include "gimbal.h"
#include "iniparser/iniparser.h"
#include "msg.h"
#include "vision/vision_main.h"
// global variable
char SERIAL_PORT[20] = "/dev/ttyACM0";
#define CONNECT_TO_SERIAL 0
Gimbal *gimbal = NULL;
rs2::pipeline *p_pipe = NULL;
std::shared_ptr<roborts_sdk::Handle> handle = nullptr;



void  start_sdk();

int main(int argc, char *argv[]){

    //
    signal(SIGINT, [](int){
        if (p_pipe) p_pipe->stop();
        _exit(0);
    });


    // Start SDK
    start_sdk();


    // Gimbal initializing
    if(CONNECT_TO_SERIAL)
        gimbal = new Gimbal(handle);
    else
        gimbal = new Gimbal(NULL);


    // New thread for vision function
    std::thread vision_main_thread = std::thread(vision_main_function);

    // Keep accepting the message from development board, the main thread services SDk
    while(1){
        if (CONNECT_TO_SERIAL)
            handle->Spin();
        usleep(1000);
    }

    return 0;
}


void start_sdk(){
    handle = std::make_shared<roborts_sdk::Handle>(SERIAL_PORT);
}