//
// Created by jsz on 12/28/19.
//

#include "armor.h"

using namespace cv;

armor::armor() {}

armor::armor(const LED_bar &left, const LED_bar &right) {
    led_bars[0] = left;
    led_bars[1] = right;
    error_angle = fabs(left.rect.angle - right.rect.angle);
    rect.width = abs(static_cast<int>(left.rect.center.x - right.rect.center.x));
    rect.height = static_cast<int>((left.rect.size.height + right.rect.size.height) / 2);
    center.x = static_cast<int>((left.rect.center.x + right.rect.center.x) / 2);
    center.y = static_cast<int>((left.rect.center.y + right.rect.center.y) / 2);
    rect.x = center.x - rect.width / 3;
    rect.y = center.y - rect.height / 3;
    rect.width *= 2.0 / 3;
    rect.height *= 2.0 / 3;
}

int armor::get_average_intensity(const Mat &img) {
    if (rect.width < 1 || rect.height < 1 || rect.x < 1 || rect.y < 1
        || rect.width + rect.x > img.cols || rect.height + rect.y > img.rows)
        return 255;
    Mat roi = img(Range(rect.y, rect.y + rect.height), Range(rect.x, rect.x + rect.width));
    average_intensity = static_cast<int>(mean(roi).val[0]);
    return average_intensity;
}

void armor::draw_rect(Mat &img, Point2f roi_offset_point) const {
    rectangle(img, rect + Point_<int>(roi_offset_point), Scalar(255, 255, 255), 1);
}

void armor::draw_spot(Mat &img, Point2f roi_offset_point) const {
    circle(img, center + Point_<int>(roi_offset_point), int(rect.height / 4), Scalar(0, 0, 255), -1);
}

bool armor::is_suitable_size() {
    auto light_dis = std::sqrt((led_bars[0].rect.center.x - led_bars[1].rect.center.x) *
                                  (led_bars[0].rect.center.x - led_bars[1].rect.center.x) +
                                  (led_bars[0].rect.center.y - led_bars[1].rect.center.y) *
                                  (led_bars[0].rect.center.y - led_bars[1].rect.center.y));

    if (led_bars[0].rect.size.height * 0.7f < led_bars[1].rect.size.height
        && led_bars[0].rect.size.height * 1.3f > led_bars[1].rect.size.height && light_dis < 575.0f) {
        float armor_width = fabs(led_bars[0].rect.center.x - led_bars[1].rect.center.x);

        if (armor_width > led_bars[0].rect.size.width
            && armor_width > led_bars[1].rect.size.width
            && armor_width > (led_bars[0].rect.size.width + led_bars[1].rect.size.width) * 3) {
            float h_max = (led_bars[0].rect.size.height + led_bars[1].rect.size.height) / 2.0f;

            // 两个灯条高度差不大
            if (fabs(led_bars[0].rect.center.y - led_bars[1].rect.center.y) < 1.0f * h_max) {
                // 长宽比判断
                if (h_max * 4.0f > rect.width && h_max < 1.1f * rect.width) {
                    return true;
                }
            }
        }
    }
    return false;
}

void armor::max_match(std::vector<LED_bar> &LEDs, size_t i, size_t j) {
    RotatedRect R, L;
    if (LEDs[0].rect.center.x > LEDs[1].rect.center.x) {
        R = LEDs[0].rect;
        L = LEDs[1].rect;
    } else {
        R = LEDs[1].rect;
        L = LEDs[0].rect;
    }
    float angle = L.angle - R.angle;
    if (angle < 1e-3f) {
        angle = 0.0f;
    }
    float factor = error_angle + 0.5 * angle;

    if (!LEDs.at(i).matched && !LEDs.at(j).matched) {

        LEDs.at(i).matched = true;
        LEDs.at(i).match_index = j;
        LEDs.at(i).match_factor = factor;

        LEDs.at(j).matched = true;
        LEDs.at(j).match_index = i;
        LEDs.at(j).match_factor = factor;
    }

    if (LEDs.at(i).matched && !LEDs.at(j).matched) {
        if (factor < LEDs.at(i).match_factor) {
            LEDs.at(LEDs.at(i).match_index).matched = false;
            LEDs.at(i).match_factor = factor;
            LEDs.at(i).match_index = j;

            LEDs.at(j).matched = true;
            LEDs.at(j).match_factor = factor;
            LEDs.at(j).match_index = i;

        }
    }
    if (LEDs.at(j).matched && !LEDs.at(i).matched) {
        if (factor < LEDs.at(j).match_factor) {
            LEDs.at(LEDs.at(j).match_index).matched = false;
            LEDs.at(j).match_factor = factor;
            LEDs.at(j).match_index = i;

            LEDs.at(i).matched = true;
            LEDs.at(i).match_factor = factor;
            LEDs.at(i).match_index = j;
        }
    }
    if (LEDs.at(j).matched && LEDs.at(i).matched
        && LEDs.at(i).match_factor > factor && LEDs.at(j).match_factor > factor) {

        LEDs.at(LEDs.at(i).match_index).matched = false;
        LEDs.at(i).matched = true;
        LEDs.at(i).match_factor = factor;
        LEDs.at(i).match_index = j;

        LEDs.at(LEDs.at(j).match_index).matched = false;
        LEDs.at(j).matched = true;
        LEDs.at(j).match_factor = factor;
        LEDs.at(j).match_index = i;
    }
}