//
// Created by jsz on 12/19/19.
//

#ifndef ROBOTBASE_AUTOAIM_H
#define ROBOTBASE_AUTOAIM_H

#include <opencv2/opencv.hpp>
#include <stdlib.h>
#include <roboBase/RobotBase/Robogrinder_SDK/serial_port.h>
#include "../control.h"
#include "armor.h"
#include "rapidjson/document.h"
#include "rapidjson/filereadstream.h"
#include "../../common.h"

class ArmorDetector {
public:
    ArmorDetector() {
        t_start_ = cv::getTickCount();
        /** get configurations from json file*/
//        FILE *fp = fopen("config.json","r");
//        char json_buffer[1000];
//        rapidjson::FileReadStream json(fp,json_buffer,sizeof(json_buffer));
//        rapidjson::Document d;
//        d.ParseStream(json);
//        fclose(fp);
//        this->OFFSET_PITCH = d["PITCH_OFFSET"].GetInt();
//        this->OFFSET_YAW = d["YAW_OFFSET"].GetInt();
    }

    ~ArmorDetector() = default;

    int armorTask(cv::Mat &img, OtherParam other_param, serial_port sp);

    bool DetectArmor(cv::Mat &img, cv::Rect roi);

public:
    int color_th_ = 13;
    int gray_th_ = 24;
    int OFFSET_INT_YAW = 1800;
    int OFFSET_INT_PITCH = 1800;

    int OFFSET_YAW;
    int OFFSET_PITCH;
private:
    bool makeRectSafe(cv::Rect &rect, const cv::Size &size) {
        if (rect.x < 0)
            rect.x = 0;
        if (rect.x + rect.width > size.width)
            rect.width = size.width - rect.x;
        if (rect.y < 0)
            rect.y = 0;
        if (rect.y + rect.height > size.height)
            rect.height = size.height - rect.y;
        return !(rect.width <= 0 || rect.height <= 0);
    }

    cv::Rect GetRoi(const cv::Mat &img);

private:

    double t_start_;

    cv::Rect last_target_;
    int lost_count = 0;
    int detect_count = 0;

    uint8_t color_{};
    uint8_t mode_{};
    uint8_t level_;
    std::vector<cv::Point2f> final_armor_2Dpoints;
    cv::Mat cameraMatrix, distCoeffs;

private:
    // 判断大小装甲板类型相关参数
    std::list<bool> history_;
    int filter_size_ = 5;
    bool is_small_{};
    // Sam prediction
    int yaw_array[3];
    int yaw_array_count = 0;
    int yaw_array_size = 1;

};


#endif //ROBOTBASE_AUTOAIM_H
