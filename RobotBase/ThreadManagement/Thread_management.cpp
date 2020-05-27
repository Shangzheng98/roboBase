//
// Created by jsz on 5/19/20.
//
#include "Thread_management.h"

using namespace std;

ThreadManagement::ThreadManagement() {
    cout << "THEAD START WORKING !!!" << endl;
    sem_init(&mode_signal, 0, 0);
    daheng = new Daheng();
    if (daheng->init() == 0) {
        printf("fail to init Daheng camera library\n");
        return ;
    }
    serialPort = serial_port(name, B115200);
    otherParam.level = 1;
    otherParam.id = 0;
    otherParam.mode = AUTOAIM;

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
        //auto time0 = static_cast<double>(getTickCount());
        cv::Mat frame;
        daheng->getImage(frame);
        buffer_lock.lock();
        if (buffer->size() == 1000) {
            delete buffer;
            buffer = new std::queue<cv::Mat>();
        }
        inputcounter++;
        buffer->push(frame);
        //cout << buffer->size() << endl;
        buffer_lock.unlock();
        //time0 = ((double) getTickCount() - time0) / getTickFrequency();
        //cout << "use time is " << time0 * 1000 << "ms" << endl;
    }

}

void ThreadManagement::AutoAim() {
    ArmorDetector armorDetector;

    while (1) {
        auto time0 = static_cast<double>(getTickCount());
        if (otherParam.mode == BIGBUFF)
        {
            cout << "auto aim thread sleep" << endl;
            sem_wait(&mode_signal);
        } else{
            sem_post(&mode_signal);
        }
        buffer_lock.lock();
        if(buffer->size()==0)
        {
            buffer_lock.unlock();
            continue;
        }

        Mat frame = buffer->front();
        buffer->pop();
        outputcounter++;
        buffer_lock.unlock();
        //auto time0 = static_cast<double>(getTickCount());
        armorDetector.armorTask(frame, otherParam, serialPort);
        time0 = ((double) getTickCount() - time0) / getTickFrequency();
        cout << "use time is " << time0 * 1000 << "ms" << endl;
    }

}
void ThreadManagement::Bigbuff() {
    BigbufDetector bigebufDetector(640,480,serialPort);
    while (1)
    {
        if (otherParam.mode == AUTOAIM)
        {
            cout << "bigbuff thread sleep"<< endl;
            sem_wait(&mode_signal);
        } else{
            sem_post(&mode_signal);
        }
        buffer_lock.lock();
        Mat frame = buffer->front();
        bigebufDetector.feed_im(frame);
        //bigebufDetector.getTest_result();

    }
    return;
}

void ThreadManagement::Communication_thread() {

}

ThreadManagement::~ThreadManagement() {
    delete daheng;
    delete buffer;
}



