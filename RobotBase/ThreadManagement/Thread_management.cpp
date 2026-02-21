//
// Created by jsz on 5/19/20.
//
#include "Thread_management.h"

using namespace std;

ThreadManagement::ThreadManagement() {
    cout << "THEAD START WORKING !!!" << endl;
    buffer.set_capacity(SIZE_);

    // QUAL-2: 从 config.json 加载标定参数，注入给检测器
    loadConfig();
    if (!cam_matrix_.empty()) {
        armorDetector.setCameraParams(cam_matrix_, dist_coeffs_);
    }

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
    while (running_) {
        cv::Mat frame;
        daheng->getImage(frame);
        buffer_lock.lock();
        buffer.push_back(frame);
        buffer_lock.unlock();
    }

}

void ThreadManagement::AutoAim() {


    while (running_) {
        //auto time0 = static_cast<double>(getTickCount());

        {
            std::unique_lock<std::mutex> lck_a(aim_mtx);
            cond_aim.wait(lck_a, [this]{ return aim_ready || !running_; });
            if (!running_) break;
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
    if (!cam_matrix_.empty()) {
        bigebufDetector.setCameraParams(cam_matrix_, dist_coeffs_);
    }

    while (running_) {

        {
            std::unique_lock<std::mutex> lck_b(buff_mtx);
            cond_buff.wait(lck_b, [this]{ return buff_ready || !running_; });
            if (!running_) break;
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
    uint8_t last_mode = otherParam.mode;
    while (running_) {
        serial_recive_data recv;
        serialPort.recive_data(recv);

        // 校验帧头
        if ((uint8_t)recv.rawData[0] != 0xaf) continue;

        // rawData 布局：[0]=head [1]=id [2]=robot_id [3]=level [4]=vison_mode
        uint8_t new_mode = (uint8_t)recv.rawData[4];
        if (new_mode == last_mode) continue;
        last_mode = new_mode;

        if (new_mode == AUTOAIM) {
            pause('b');
            sleep(1);
            resume('a');
        } else if (new_mode == BIGBUFF) {
            pause('a');
            sleep(1);
            resume('b');
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
     // 切换模式前清空缓冲区，防止新检测器处理 sleep(1) 期间积累的旧帧
     buffer_lock.lock();
     buffer.clear();
     buffer_lock.unlock();

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
    running_ = false;
    cond_aim.notify_all();   // 唤醒阻塞在条件变量上的线程，使其检查 running_ 后退出
    cond_buff.notify_all();
}
void ThreadManagement::loadConfig() {
    // QUAL-2: 用 OpenCV FileStorage 读取 config.json（JSON 格式支持 OpenCV 3.1+）
    cv::FileStorage fs("config.json", cv::FileStorage::READ | cv::FileStorage::FORMAT_JSON);
    if (!fs.isOpened()) {
        printf("Warning: config.json not found, using hardcoded camera params\n");
        return;
    }

    std::vector<double> cmVals, dcVals;
    cv::FileNode cmNode = fs["cameraMatrix"];
    for (auto it = cmNode.begin(); it != cmNode.end(); ++it)
        cmVals.push_back(static_cast<double>(*it));

    cv::FileNode dcNode = fs["distCoeffs"];
    for (auto it = dcNode.begin(); it != dcNode.end(); ++it)
        dcVals.push_back(static_cast<double>(*it));

    if (cmVals.size() == 9 && dcVals.size() == 5) {
        cam_matrix_ = cv::Mat(3, 3, CV_64F, cmVals.data()).clone();
        dist_coeffs_ = cv::Mat(1, 5, CV_64F, dcVals.data()).clone();
        printf("Camera params loaded from config.json\n");
    } else {
        printf("Warning: invalid camera params in config.json, using hardcoded defaults\n");
    }
    fs.release();
}

ThreadManagement::~ThreadManagement() {
    delete daheng;
}





