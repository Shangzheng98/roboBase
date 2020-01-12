//
// Created by jsz on 12/28/19.
//

#ifndef ROBOTBASE_ARMOR_H
#define ROBOTBASE_ARMOR_H

#include <opencv2/opencv.hpp>

class LED_bar {
public:
    LED_bar() : matched(false) {}

    LED_bar(const cv::RotatedRect &R) {
        rect.angle = R.angle;
        rect.center = R.center;
        rect.size = R.size;
        matched = false;
    }

    cv::RotatedRect rect;
    bool matched;
    size_t match_index;
    float match_factor;
};

class armor {
public:
    armor();

    armor(const LED_bar &left, const LED_bar &right);

    void draw_rect(cv::Mat &img, cv::Point2f roi_offset_point) const;

    void draw_spot(cv::Mat &img, cv::Point2f roi_offset_point) const;

    int get_average_intensity(const cv::Mat &img);

    void max_match(std::vector<LED_bar> &LEDs, size_t i, size_t j) ;

    bool is_suitable_size();

    LED_bar led_bars[2];
    float error_angle;
    cv::Point2i center;
    cv::Rect2i rect;
    int average_intensity;

};

#endif //ROBOTBASE_ARMOR_H
