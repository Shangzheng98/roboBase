
#include <thread>
#include <cstdio>
#include "vision/vision_main.h"


// global variable
char SERIAL_PORT[20] = "/dev/serial_sdk";
#define CONNECT_TO_SERIAL 0

using namespace cv;
int main(int argc, char *argv[]) {

#if CONNECT_TO_SERIAL
#else
    //gimbal = new Gimbal(nullptr);
#endif

    // New thread for vision function
    std::thread vision_main_thread = std::thread(vision_main_function);

    //Keep accepting the message from development board, the main thread services SDk
    vision_main_thread.join();
    return 0;
}


