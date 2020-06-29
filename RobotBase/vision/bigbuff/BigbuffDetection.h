//
// Created by eric on 2/15/20.
//

#ifndef ROBOTBASE_BIGBUFFDETECTION_H
#define ROBOTBASE_BIGBUFFDETECTION_H

#include "opencv2/opencv.hpp"
#include <ctime>
#include <roboBase/RobotBase/Robogrinder_SDK/serial_port.h>
#include <roboBase/RobotBase/Robogrinder_SDK/message.h>
#include "pred_algrsm.h"

#define DEBUG 1
#if DEBUG
#define DISP(a) std::cout<<a<<std::endl
#define SHOW_IM(name, im) cv::imshow(name,im)
#else
#define DISP(a)
#define SHOW_IM(name,im)
#endif

#define LEN_RATIO(a, b) (a>b)?(a/b):(b/a)

struct frame_info {

    clock_t f_time;
    cv::Point2f center;
    cv::Point2f angular_vector;
    std::vector<cv::Point2f> armor_points;

};


//////////////////////////////////////////////////

class BigbufDetector {

public:

    int OFFSET_YAW = 3600;
    int OFFSET_PITCH = 3600;

    // constructor: information about camera
    BigbufDetector(int cols, int rows, serial_port sp);

    ~BigbufDetector();

    void feed_im(cv::Mat &input_image);

    void getTest_result();

    void make_prediction();

public:
    int color_th_ = 130;//131
    int gray_th_ = 25;//24


private:
    int IMAGE_COLS;
    int IMAGE_ROWS;
    cv::Mat image;
    cv::Mat RGBim;
    serial_port SP;
    int sample_num = 0;
    uint8_t color_ = 1;
    clock_t t_ms = clock();
    std::vector<cv::Point3f> real_armor_points;
    const int THR_R_min = 32, THR_G_min = 43, THR_B_min = 22, THR_R_max = 255, THR_G_max = 255, THR_B_max = 255;
    const int THR_S_min = 0, THR_S_max = 100, THR_V_min = 0, THR_V_max = 100;

    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 1293.5303221625442802, 0.3651215140945823, 355.9091806402759630,
            0.0000000000000000, 1293.9256252855957428, 259.1868664367483461,
            0.0000000000000000, 0.0000000000000000, 1.0000000000000000);

    cv::Mat distCoeffs = (cv::Mat_<double>(1, 5)
            << -0.2126367859619807, 0.2282910064864265, 0.0020583387355406, 0.0006136511397638, -0.7559987171745171);

    std::vector<frame_info> frames;


    void refresh_frames();

    void filte_image(cv::Mat &im);

    bool locate_target(cv::Mat &im);

    void record_info(frame_info frameInfo);

    bool contour_valid(std::vector<cv::Point> &contour);

    void drawContour_Rec(cv::RotatedRect rect, cv::Mat &src);


};


#endif //ROBOTBASE_BIGBUFFDETECTION_H
