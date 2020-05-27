//
// Created by jsz on 5/19/20.
//

#ifndef ROBOTBASE_THREAD_MANAGEMENT_H
#define ROBOTBASE_THREAD_MANAGEMENT_H

#include <stdlib.h>
#include <boost/thread/mutex.hpp>
#include "../vision/autoAim/autoAim.h"
#include "../vision/bigbuff/BigbuffDetection.h"
#include "../vision/cam/Daheng.h"
#include "../Robogrinder_SDK/serial_port.h"
#include "../common.h"
#include <semaphore.h>
class ThreadManagement {
public:
    ThreadManagement();
    ~ThreadManagement();
    void ImageProduce();

    void Bigbuff();

    void AutoAim();

    void Communication_thread();
    int inputcounter =0;
    int outputcounter = 0;

private:
    char name[20] = "/dev/ttyUSB0";
    std::queue<cv::Mat> *buffer = new std::queue<cv::Mat>();
    boost::mutex buffer_lock;
    sem_t mode_signal;
    OtherParam otherParam;
    serial_port serialPort;
    Daheng *daheng;

};

#endif //ROBOTBASE_THREAD_MANAGEMENT_H
