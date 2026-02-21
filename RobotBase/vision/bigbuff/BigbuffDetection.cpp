//
// Created by eric on 2/15/20.
//

#include "BigbuffDetection.h"


BigbufDetector::BigbufDetector(serial_port &sp) {
    /// solvepnp Data
    x = -width / 2;
    y = height / 2;
    z = 0;
    this->real_armor_points.emplace_back(x, y, z);
    x = width / 2;
    y = height / 2;
    z = 0;
    this->real_armor_points.emplace_back(x, y, z);
    x = width / 2;
    y = -height / 2;
    z = 0;
    this->real_armor_points.emplace_back(x, y, z);
    x = -width / 2;
    y = -height / 2;
    z = 0;
    this->real_armor_points.emplace_back(x, y, z);
    this->sp = sp;
    this->frame_buffer.set_capacity(10);  // BUG-2: initialize buffer capacity
    // PERF: 预计算形态学核，避免 filte_image 每帧重新分配
    this->dilate_kernel_5_ = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    this->erode_kernel_3_  = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::createTrackbar("offset_YAW", "offset", &this->OFFSET_YAW, 3600);
    cv::createTrackbar("offset_PITCH", "offset", &this->OFFSET_PITCH, 3600);
}


void BigbufDetector::filte_image(cv::Mat &im,uint8_t color) {
    cv::Mat gray;
    cvtColor(im, gray, cv::COLOR_BGR2GRAY);

    std::vector<cv::Mat> BGR_channels;
    split(im, BGR_channels);

    this->RGBim = im;
    cv::Mat result_img;
    if (color == 0) // opposite red
    {
        subtract(BGR_channels[2], BGR_channels[1], result_img);
    } else {
        subtract(BGR_channels[0], BGR_channels[2], result_img);
    }
    cv::Mat binary_brightness_img, binary_color_img;
    threshold(gray, binary_brightness_img, gray_th_, 255, cv::THRESH_BINARY);
    threshold(result_img, binary_color_img, color_th_, 255, cv::THRESH_BINARY);
    ///test:
    //SHOW_IM("binary_brightness_img", binary_brightness_img);
    //SHOW_IM("binary_color_img", binary_color_img);
    im = binary_color_img & binary_brightness_img;

    // PERF: 使用预计算核，避免每帧重复调用 getStructuringElement
    cv::dilate(im, im, dilate_kernel_5_);
    cv::dilate(im, im, cv::Mat());
    cv::erode(im, im, erode_kernel_3_);
}


void BigbufDetector::drawContour_Rec(cv::RotatedRect rect, cv::Mat &src) {

    //cv::Point2f *vertices = new cv::Point2f[4];
    cv::Point2f vertices[4];
    rect.points(vertices);

    for (int j = 0; j < 4; j++)
        cv::line(src, vertices[j], vertices[(j + 1) % 4], cv::Scalar(180, 180, 180), 1);

}

bool BigbufDetector::contour_valid(std::vector<cv::Point> &contour) {
    ///// TO DO: find suitable threshold for area
    float area = cv::contourArea(contour);
    if (area < 1000)
        return false;

    cv::RotatedRect rect = cv::minAreaRect(contour);
    ///// notice that l_ratio is always bigger than 0
    float l_ratio;
    l_ratio = LEN_RATIO(rect.size.height, rect.size.width);
    if (l_ratio < 1.9 || 2.2 < l_ratio)
        return false;


    float a_ratio = rect.size.area() / area;
    ///test///std::cout<< "AREA RATIO:  "<<a_ratio<<std::endl;
    /// TO DO: set threshold
    if (a_ratio < 1.9 || a_ratio > 2.25)
        return false;

    ///test:
    drawContour_Rec(rect, image);

    return true;
}


bool BigbufDetector::locate_target(cv::Mat &im) {


    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;  // QUAL-9: 修正拼写错误

    cv::findContours(im, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);


    for (int ind = 0; ind < contours.size(); ind++) {
        if (contour_valid(contours[ind])) {
            cv::RotatedRect rect = cv::minAreaRect(contours[ind]);
            cv::Moments M = cv::moments(contours[ind]);

            ////// TO DO: find target in the plate
            float cx = M.m10 / M.m00;
            float cy = M.m01 / M.m00;
            cv::Point2f mass_ctr = cv::Point(cx, cy);

            cv::Vec4f line;
            cv::fitLine(contours[ind], line, cv::DIST_L2, 0, 0.01, 0.01);
            cv::Point2f direc = cv::Point2f(line[0], line[1]);

            cv::Point2f v = rect.center - mass_ctr;
            // PERF: fitLine 返回的方向向量已是单位向量，无需再做归一化
            v = (direc.dot(v) > 0) ? (direc) : (-direc);
            float long_side = (rect.size.height > rect.size.width) ? (rect.size.height) : (rect.size.width);
            cv::Point2f c_R = mass_ctr + (long_side * 13 / 14) * v;
            cv::Point2f a_vector = mass_ctr - (long_side * 1 / 5) * v;

            cv::circle(im, c_R, 4, cv::Scalar(255, 255, 255), 2);
            cv::circle(im, a_vector, 1, cv::Scalar(255, 255, 255), 1);

            if (hierarchy[ind][2] == -1) {
                std::cout << "None" << std::endl;
                return false;
            }

            cv::RotatedRect inner_rect = cv::minAreaRect(contours[hierarchy[ind][2]]);
            cv::ellipse(this->RGBim, inner_rect, cv::Scalar(0, 0, 255), 3);
            SHOW_IM("RGB", this->RGBim);
            cv::Point2f rect_points[4];
            inner_rect.points(rect_points);
            std::vector<cv::Point2f> armor_points;
            for (int n = 0; n < 4; n++) armor_points.push_back(rect_points[n]);

            // ** record the info of the target at the moment ** //
            frame_info target;
            target.center = c_R;
            target.angular_vector = a_vector;
            target.f_time = clock();
            target.armor_points = armor_points;
            frame_buffer.push_back(target);  // BUG-2: push to frame_buffer
            return true;
        }
    }
    return false;
}



void BigbufDetector::feed_im(cv::Mat &input_image,OtherParam othter_param) {

    // check
//    if(  input_image.cols != this->IMAGE_COLS || input_image.rows != this->IMAGE_ROWS)
//          return;

    this->image = input_image;
    this->otherParam = othter_param;
    filte_image(this->image, otherParam.color);

    if (! locate_target(this->image) || ! this->frame_buffer.full())
        return;

    make_prediction();

#if DEBUG
    cv::imshow("bigbuff", this->image);
    cv::waitKey(1);
#endif
}


void BigbufDetector::make_prediction() {
    std::vector<cv::Point2f> armor_points = this->frame_buffer[0].armor_points;
    cv::Mat rvec, tvec;
    cv::solvePnP(this->real_armor_points, armor_points, cameraMatrix, distCoeffs, rvec, tvec);
    cv::Point3f target_3d;
    target_3d = cv::Point3f(tvec);
    int pitch = int((atan2(target_3d.y - 80, target_3d.z) + (float) (OFFSET_PITCH * CV_PI / 1800)) * 0.6 * 10000);
    int yaw = int((-atan2(target_3d.x, target_3d.z) + (float) (OFFSET_YAW * CV_PI / 1800)) * 0.6 * 10000);  // BUG-1: was OFFSET_PITCH
    DISP("yaw" << yaw);
    DISP("pitch" << pitch);
    serial_gimbal_data data;
    data.size = 6;
    data.rawData[0] = data.head;
    data.rawData[1] = data.id;
    data.rawData[2] = pitch;
    data.rawData[3] = pitch >> 8;
    data.rawData[4] = yaw;
    data.rawData[5] = yaw >> 8;

    this->sp.send_data(data);
}

