//
// Created by eric on 2/15/20.
//

#ifndef ROBOTBASE_BIGBUFFDETECTION_H
#define ROBOTBASE_BIGBUFFDETECTION_H
#include "opencv2/opencv.hpp"
#include <time.h>

struct frame_info{

    clock_t f_time;
    cv::Point2f position;

};


//////////////////////////////////////////////////

class BigbufDetection {


private:
    int IMAGE_COLS;
    int IMAGE_ROWS;
    cv::Mat image;
    bool test;
    const int THR_R_min = 32, THR_G_min = 43, THR_B_min = 22, THR_R_max = 255, THR_G_max = 255, THR_B_max = 255;
    const int THR_S_min = 0,THR_S_max = 100,THR_V_min = 0, THR_V_max = 100;

    std::vector<frame_info> frames;


    void refresh_frames();

    void filte_image(cv::Mat &im);

    cv::Point2f locate_target(cv::Mat &im);

    void record_info(clock_t t, cv::Point2f& pos);

    bool contour_valid(std::vector<cv::Point>& contour);

    void drawContour_Rec(cv::RotatedRect rect, cv::Mat &src);


public:
    // constructor: information about camera
    BigbufDetection(int cols, int rows, bool test);

    void feed_im(cv::Mat& input_image);

    void getTest_result();

    void make_prediction();
};


#endif //ROBOTBASE_BIGBUFFDETECTION_H
