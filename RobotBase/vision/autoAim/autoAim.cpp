//
// Created by jsz on 12/19/19.
//

#include "autoAim.h"

using namespace cv;
using namespace std;

double calc_distance(Point2f p1, Point2f p2) {
    return pow(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2), 0.5);
}


cv::Rect ArmorDetector::GetRoi(const cv::Mat &img) {
    Size img_size = img.size();
    Rect rect_tmp = last_target_;
    Rect rect_roi;
    if (rect_tmp.x == 0 || rect_tmp.y == 0
        || rect_tmp.width == 0 || rect_tmp.height == 0
        || lost_count >= 15) {
        last_target_ = Rect(0, 0, img_size.width, img_size.height);
        rect_roi = Rect(0, 0, img_size.width, img_size.height);
        return rect_roi;
    } else {
        float scale = 2;
        if (lost_count <= 35)
            scale = 2.3;
        else if (lost_count <= 60)
            scale = 3;
        else if (lost_count <= 100)
            scale = 3.5;

        int w = int(rect_tmp.width * scale);
        int h = int(rect_tmp.height * scale);
        int x = int(rect_tmp.x - (w - rect_tmp.width) * 0.5f);
        int y = int(rect_tmp.y - (h - rect_tmp.height) * 0.5f);

        rect_roi = Rect(x, y, w, h);

        if (!makeRectSafe(rect_roi, img_size)) {
            rect_roi = Rect(0, 0, img_size.width, img_size.height);
        }
    }
    return rect_roi;
}

bool ArmorDetector::DetectArmor(cv::Mat &img, cv::Point3f &target_3d, cv::Rect roi) {
    Mat roi_image = img(roi);
    Point2f offset_roi_point(roi.x, roi.y);
    vector<LED_bar> LED_bars;
    Mat binary_brightness_img, binary_color_img, gray, debug_img;
    debug_img = img.clone();
    cvtColor(roi_image, gray, COLOR_BGR2GRAY);
//    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));
//    dilate(roi_image, roi_image, element);
    vector<Mat> BGR_channels;
    split(roi_image, BGR_channels);
    Mat result_img;
    if (color_ == 0) // opposite red
    {
        subtract(BGR_channels[2], BGR_channels[1], result_img);
    } else {
        subtract(BGR_channels[0], BGR_channels[2], result_img);
    }

    threshold(gray, binary_brightness_img, gray_th_, 255, THRESH_BINARY);
    threshold(result_img, binary_color_img, color_th_, 255, THRESH_BINARY);
#if SHOW_BINART
    imshow("binary_brightness_img", binary_brightness_img);
    imshow("binary_color_img", binary_color_img);
#endif
    vector<vector<Point> > contours_light;
    vector<vector<Point> > contours_brightness;
    findContours(binary_color_img, contours_light, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    findContours(binary_brightness_img, contours_brightness, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    for (unsigned int i = 0; i < contours_brightness.size(); i++) {
        double area = contourArea(contours_brightness[i]);
        if (area > 1e5) {
            continue;
        }
        for (unsigned int j = 0; j < contours_light.size(); j++) {
            if (pointPolygonTest(contours_light[j], contours_brightness[i][0], false) >= 0.0) {
                double length = arcLength(contours_brightness[i], true); // 灯条周长
                if (length > 20 && length < 4000) {
                    RotatedRect RRect = fitEllipse(contours_brightness[i]);
                    //RotatedRect RRect = minAreaRect(contours_brightness[i]);
                    auto light_aspect_ratio =
                            std::max(RRect.size.width, RRect.size.height) /
                            std::min(RRect.size.width, RRect.size.height);
                    //auto light_aspect_ratio = RRect.size.height / RRect.size.width;
                    if (RRect.angle > 90.0f)
                        RRect.angle = RRect.angle - 180.0f;

                    //(fabs(RRect.angle)<30||(fabs(RRect.angle)<90 && fabs(RRect.angle)>70))&&
                    if (fabs(RRect.angle) < 30) {
#if SHOW_LIGHT_CONTOURS
                        char temp[20];
                        sprintf(temp, "%0.2f", light_aspect_ratio);
                        putText(debug_img, temp, RRect.center + Point2f(0, -40) + offset_roi_point,
                                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
                        Point2f rect_point[4];
                        RRect.points(rect_point);
                        for (int i = 0; i < 4; i++) {
                            line(debug_img, rect_point[i] + offset_roi_point,
                                 rect_point[(i + 1) % 4] + offset_roi_point,
                                 Scalar(255, 0, 255), 1);
                        }
#endif

#if SHOW_LIGHT_CONTOURS
                        char temp1[20];
                        sprintf(temp1, "%0.2f", RRect.angle);
                        putText(debug_img, temp1, RRect.center + Point2f(0, -10) + offset_roi_point,
                                FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 0), 1);
#endif
                        LED_bar r(RRect);
                        LED_bars.emplace_back(r);

                    }
                }
                break;
            }
        }
    }
//    //==========================================possible armor=========================================
    printf("led bar number: %zu\n", LED_bars.size());
    for (size_t i = 0; i < LED_bars.size(); i++) {
        for (size_t j = i + 1; j < LED_bars.size(); j++) {
            armor temp_armor(LED_bars.at(i), LED_bars.at(j));
            if (temp_armor.error_angle < 7.0f) {
                if (temp_armor.is_suitable_size()) {
                    if (temp_armor.get_average_intensity(gray) < 70) {
                        temp_armor.max_match(LED_bars, i, j);
                    }
                }
            }
        }
    }
//
    //====================================find final armors============================================
    vector<armor> final_armor_list;
    for (size_t i = 0; i < LED_bars.size(); i++) {
        if (LED_bars.at(i).matched) {
            LED_bars.at(LED_bars.at(i).match_index).matched = false; //clear another matching flag
            armor arm_tmp(LED_bars.at(i), LED_bars.at(LED_bars.at(i).match_index));
            //arm_tmp.draw_rect(img, offset_roi_point);
            final_armor_list.push_back(arm_tmp);
        }
    }
    printf("final armor size %zu\n", final_armor_list.size());
//
//
    float dist = 1e8;
    bool found_flag = false;
    armor target;
    Point2f roi_center(roi.width / 2, roi.height / 2);
    float dx, dy;
    for (auto &i : final_armor_list) {
#if FAST_DISTANCE
        dx = fabs(final_armor_list.at(i).center.x - roi_center.x);
        dy = fabs(final_armor_list.at(i).center.y - roi_center.y);
#else
        dx = pow((i.center.x - roi_center.x), 2.0f);
        dy = pow((i.center.y - roi_center.y), 2.0f);
#endif
        if (dx + dy < dist) {
            target = i;
            dist = dx + dy;
        }
#if SHOW_FINAL_ARMOR
        i.draw_rect(debug_img, offset_roi_point);

#endif
        found_flag = true;
    }
#if SHOW_ROI
    rectangle(debug_img, roi, Scalar(255, 0, 255), 1);
#endif
    if (found_flag) {
#if SHOW_DRAW_SPOT
        target.draw_spot(img, offset_roi_point);
#endif
        Point2f point_tmp[4];
        Point2f point_2d[4];
        // 左右灯条分类，本别提取装甲板四个外角点
        RotatedRect R, L;
        if (target.led_bars[0].rect.center.x > target.led_bars[1].rect.center.x) {
            R = target.led_bars[0].rect;
            L = target.led_bars[1].rect;
        } else {
            R = target.led_bars[1].rect;
            L = target.led_bars[0].rect;
        }
        L.points(point_tmp);
        point_2d[0] = point_tmp[1];
        point_2d[3] = point_tmp[0];
        R.points(point_tmp);
        point_2d[1] = point_tmp[2];
        point_2d[2] = point_tmp[3];
        vector<Point2f> points_roi_tmp;
        for (int i = 0; i < 4; i++) {
            points_roi_tmp.push_back(point_2d[i] + offset_roi_point);
        }
        float armor_h = target.rect.height;
        float armor_w = target.rect.width;
        is_small_ = armor_w / armor_h < 3.3f;

        //计算ROI的相关参数
        last_target_ = boundingRect(points_roi_tmp);
#if SHOW_LAST_TARGET
        rectangle(debug_img, last_target_, Scalar(255, 255, 255), 1);

#endif
        lost_count = 0;
        target_3d.x = last_target_.x + (last_target_.width) / 2;
        target_3d.y = last_target_.y + (last_target_.height) / 2;
    } else {
        lost_count++;
    }
    detect_count++;

    imshow("debug_img", debug_img);
    return found_flag;
}

int ArmorDetector::armorTask(cv::Mat &color, OtherParam other_param) {
    color_ = other_param.color;
    mode_ = other_param.mode;
    level_ = other_param.level;

#if ROI_ENABLE
    Rect roi = GetRoi(color);
#else
    Size img_size = color.size();
    Rect roi = Rect(0, 0, img_size.width, img_size.height);
#endif
    Point3f target_3d = {0, 0, 0};

    DetectArmor(color, target_3d, roi);
    float target_2d_color_pixel[2] = {target_3d.x, target_3d.y};
    float target_2d_depth_pixel[2] = {};
    float world_target_3d_temp[3] = {};
    Point3f world_target_3d;
//    rs2_project_color_pixel_to_depth_pixel(target_2d_depth_pixel,
//                                           reinterpret_cast<const uint16_t *>(frames.get_depth_frame().get_data()),
//                                           0.001, 0, 10, &depth_intrinsics, &color_intrinsics,
//                                           &color_extrin_to_depth, &depth_extrin_to_color, target_2d_color_pixel);

    //auto dis = frames.get_depth_frame().get_distance(target_2d_depth_pixel[0],target_2d_depth_pixel[1]);
    //rs2_deproject_pixel_to_point(world_target_3d_temp, &depth_intrinsics, target_2d_color_pixel, dis);


    //world_target_3d = {(world_target_3d_temp[0]) / 10, world_target_3d_temp[1] / 10, (world_target_3d_temp[2]) / 10};
//    printf("target_2d_color_pixel X: %f, Y:%f\n ", target_2d_color_pixel[0], target_2d_color_pixel[1]);
//    printf("target_2d_depth_pixel X: %f, Y:%f\n ", target_2d_depth_pixel[0], target_2d_depth_pixel[1]);
//    printf("未转化坐标深度: %0.2f\n",frames.get_depth_frame().get_distance(target_2d_color_pixel[0],target_2d_color_pixel[1]));
//    printf("转化后坐标深度： %0.2f\n",dis);
    //gimbal->AimAtArmor(world_target_3d, true);
}


