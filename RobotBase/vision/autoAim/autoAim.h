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
//#include "rapidjson/document.h"
//#include "rapidjson/filereadstream.h"
#include "../../common.h"

class ArmorDetector {
public:
    ArmorDetector(){
        printf(" armor detector initing!\n");
        t_start_ = cv::getTickCount();
        float x, y, z, small_width = 140, small_height = 60, big_width =230, big_height = 60;
        x = -small_width / 2;
        y = small_height / 2;
        z = 0;
        small_real_armor_points.emplace_back(x, y, z);
        x = small_width / 2;
        y = small_height / 2;
        z = 0;
        small_real_armor_points.emplace_back(x, y, z);
        x = small_width / 2;
        y = -small_height / 2;
        z = 0;
        small_real_armor_points.emplace_back(x, y, z);
        x = -small_width / 2;
        y = -small_height / 2;
        z = 0;
        small_real_armor_points.emplace_back(x, y, z);

        //**********************************************************************//
        x = -big_width / 2;
        y = big_height / 2;
        z = 0;
        big_real_armor_points.emplace_back(x, y, z);
        x = big_width / 2;
        y = big_height / 2;
        z = 0;
        big_real_armor_points.emplace_back(x, y, z);
        x = big_width / 2;
        y = -big_height / 2;
        z = 0;
        big_real_armor_points.emplace_back(x, y, z);
        x = -big_width / 2;
        y = -big_height / 2;
        z = 0;
        big_real_armor_points.emplace_back(x, y, z);
    }
    ~ArmorDetector() = default;

    int armorTask(cv::Mat &img, OtherParam other_param, serial_port sp);

    bool DetectArmor(cv::Mat &img, const cv::Rect& roi);

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
    cv::Mat cameraMatrix= (cv::Mat_<double>(3, 3) << 1293.5303221625442802, 0.3651215140945823, 355.9091806402759630,
    0.0000000000000000, 1293.9256252855957428, 259.1868664367483461,
    0.0000000000000000, 0.0000000000000000, 1.0000000000000000);;
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5)
            << -0.2126367859619807, 0.2282910064864265, 0.0020583387355406, 0.0006136511397638, -0.7559987171745171);;

private:
    // 判断大小装甲板类型相关参数
    std::list<bool> history_;
    int filter_size_ = 5;
    bool is_small_{};
    // Sam prediction
    int yaw_array[3];
    int yaw_array_count = 0;
    int yaw_array_size = 1;


    std::vector<cv::Point3f> small_real_armor_points;
    std::vector<cv::Point3f> big_real_armor_points;
};


#endif //ROBOTBASE_AUTOAIM_H
