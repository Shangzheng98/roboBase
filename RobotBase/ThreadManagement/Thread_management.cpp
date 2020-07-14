//
// Created by jsz on 5/19/20.
//
#include "Thread_management.h"

using namespace std;

ThreadManagement::ThreadManagement() {
    cout << "THEAD START WORKING !!!" << endl;
    buffer.set_capacity(SIZE_);
    daheng = new Daheng();
    if (daheng->init() == 0) {
        printf("fail to init Daheng camera library\n");
        return;
    }
    serialPort = serial_port(name, B115200);
    otherParam.level = 1;
    otherParam.id = 0;
    otherParam.mode = BIGBUFF;

    if (otherParam.id == 13) // blue 3
    {
        otherParam.color = 0; //opposite is red
    } else {
        otherParam.color = 1;
    }
}

void ThreadManagement::ImageProduce() {
    cout << " ------  CAMERA PRODUCE TASK ON !!! ------ " << endl;
    while (1) {
        cv::Mat frame;
        daheng->getImage(frame);
        buffer_lock.lock();
        buffer.push_back(frame);
        buffer_lock.unlock();
    }

}

void ThreadManagement::AutoAim() {


    while (1) {
        //auto time0 = static_cast<double>(getTickCount());

        while(!aim_ready)
        {
            printf("aim wait\n");
            std::unique_lock<std::mutex> lck_a(aim_mtx);
            cond_aim. wait(lck_a);
        }
        printf("aim work\n");

        buffer_lock.lock();
        if (buffer.empty()) {
            buffer_lock.unlock();
            continue;
        }

        Mat frame = buffer.front();
        buffer.pop_front();
        buffer_lock.unlock();
        //auto time0 = static_cast<double>(getTickCount());
        armorDetector.armorTask(frame, otherParam, serialPort);

        //time0 = ((double) getTickCount() - time0) / getTickFrequency();
        //cout << "use time is " << time0 * 1000 << "ms" << endl;
    }

}

void ThreadManagement::Bigbuff() {
    BigbufDetector bigebufDetector(serialPort);

    while (1) {

        while(!buff_ready)
        {
            printf("buff wait\n");
            //sleep(1);
            std::unique_lock<std::mutex> lck_b(buff_mtx);
            cond_buff.wait(lck_b);
        }
        printf("buff work\n");
        //sleep(2);
        buffer_lock.lock();
        if (buffer.empty()) {
            buffer_lock.unlock();
            continue;
        }
        Mat frame = buffer.front();
        buffer.pop_front();
        buffer_lock.unlock();
        bigebufDetector.feed_im(frame,otherParam);
        //bigebufDetector.getTest_result();

    }
}

void ThreadManagement::Communication_thread() {
    int c;
    while ((c = getchar()) != 'q')
    {
        switch(c)
        {
            case 'a':
                pause('b');
                sleep(1);
                resume('a');
                break;
            case 'b':
                pause('a');
                sleep(1);
                resume('b');
                break;
        }
    }

}
 void ThreadManagement::pause(char x) {
    if (x == 'a')
    {
        std::unique_lock <std::mutex> lck(aim_mtx);
        aim_ready = false;
        cond_aim.notify_one();
    }
    else
    {
        std::unique_lock <std::mutex> lck(buff_mtx);
        buff_ready = false;
        cond_buff.notify_one();
    }
}

 void ThreadManagement::resume(char x) {
     if (x == 'a')
     {
         std::unique_lock <std::mutex> lck(aim_mtx);
         aim_ready = true;
         cond_aim.notify_one();
     }
     else
     {
         std::unique_lock <std::mutex> lck(buff_mtx);
         buff_ready = true;
         cond_buff.notify_one();
     }
}

 void ThreadManagement::stop() {

}
ThreadManagement::~ThreadManagement() {
    delete daheng;
}





