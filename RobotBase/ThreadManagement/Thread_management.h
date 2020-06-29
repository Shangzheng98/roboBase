//
// Created by jsz on 5/19/20.
//

#ifndef ROBOTBASE_THREAD_MANAGEMENT_H
#define ROBOTBASE_THREAD_MANAGEMENT_H

#include <stdlib.h>
#include <boost/thread/mutex.hpp>
#include <boost/circular_buffer.hpp>
#include "../vision/autoAim/autoAim.h"
#include "../vision/bigbuff/BigbuffDetection.h"
#include "../vision/cam/Daheng.h"
#include "../Robogrinder_SDK/serial_port.h"
#include "../common.h"
#include <semaphore.h>
#include <condition_variable>

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
    void pause(char x);
    void resume(char x);
    void stop();
    char name[20] = "/dev/ttyUSB0";
    const int SIZE_ = 30;
    boost::circular_buffer<cv::Mat> buffer;
    boost::mutex buffer_lock;
    std:: mutex m_;
    sem_t mode_signal;
    OtherParam otherParam;
    serial_port serialPort;
    Daheng *daheng;
    std::mutex aim_mtx;
    std::mutex buff_mtx;
    std::condition_variable cond_aim;
    std::condition_variable cond_buff;
    bool aim_ready = false;
    bool buff_ready = false;

};

#endif //ROBOTBASE_THREAD_MANAGEMENT_H
