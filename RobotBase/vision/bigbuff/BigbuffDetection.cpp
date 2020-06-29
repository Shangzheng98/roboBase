//
// Created by eric on 2/15/20.
//

#include "BigbuffDetection.h"


BigbufDetector::BigbufDetector(int cols, int rows,serial_port sp){
    this->IMAGE_COLS = cols;
    this->IMAGE_ROWS = rows;
    this->SP = sp;

    /// solvepnp Data
    float x, y, z, width = 140.0f, height = 60.0f;

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
    cv::createTrackbar("offset_YAW","offset",&this->OFFSET_YAW,3600);
    cv::createTrackbar("offset_PITCH","offset",&this->OFFSET_PITCH,3600);
}

void BigbufDetector::refresh_frames() {

    // clear the memory
    if(this->frames.size() > 3){
        frames.erase(frames.end());
    }

    // if the time interval between first two consecutive frames is to large
    // then erase all except for the first one.
    if(this->frames.size()>2 && (this->frames[0].f_time-this->frames[1].f_time) > 0.1 ){
        frames.erase(frames.begin()+1,frames.end());
    }
    ////TO DO: refresh when angle between two frames are too large.
}

void BigbufDetector::filte_image(cv::Mat &im) {

    cv::Mat binary_brightness_img, binary_color_img, gray;
    cv::Mat ref_im;
    cvtColor(im, gray, cv::COLOR_BGR2GRAY);
    cv::Mat result_img;
    std::vector<cv::Mat> BGR_channels;
    split(im, BGR_channels);
    ///test:
    //SHOW_IM("Color",im);
    this->RGBim = im;
    if (this->color_ == 0) // opposite red
    {
        subtract(BGR_channels[2], BGR_channels[1],result_img);
    } else {
        subtract(BGR_channels[0], BGR_channels[2], result_img);
    }
    threshold(gray, binary_brightness_img, gray_th_, 255, cv::THRESH_BINARY);
    threshold(result_img, binary_color_img, color_th_, 255, cv::THRESH_BINARY);
    ///test:
    //SHOW_IM("binary_brightness_img", binary_brightness_img);
    //SHOW_IM("binary_color_img", binary_color_img);
    im = binary_color_img & binary_brightness_img;

    cv::dilate(im,im,getStructuringElement(0, cv::Size(5, 5)));
    cv::dilate(im,im,cv::Mat());
    cv::erode(im,im,getStructuringElement(0, cv::Size(3, 3)));
}


void BigbufDetector::drawContour_Rec(cv::RotatedRect rect, cv::Mat &src) {

    //cv::Point2f *vertices = new cv::Point2f[4];
    cv::Point2f vertices[4];
    rect.points(vertices);

    for (int j = 0; j < 4; j++)
    cv::line(src, vertices[j] , vertices[(j + 1) % 4] , cv::Scalar(180, 180, 180), 1);

}

bool  BigbufDetector::contour_valid(std::vector<cv::Point>& contour){
    ///// TO DO: find suitable threshold for area
    float area = cv::contourArea(contour);
    if(area<1000)
        return false;

    cv::RotatedRect rect = cv::minAreaRect(contour);
    ///// notice that l_ratio is always bigger than 0
    float l_ratio;
    l_ratio = LEN_RATIO(rect.size.height,rect.size.width);
    if(l_ratio<1.9 || 2.2< l_ratio)
        return false;


    float a_ratio = rect.size.area()/area;
    ///test///std::cout<< "AREA RATIO:  "<<a_ratio<<std::endl;
    /// TO DO: set threshold
    if(a_ratio < 1.9 || a_ratio>2.25)
        return false;

    ///test:
    drawContour_Rec(rect, image);
    //DISP(this->sample_num<< "   "<<l_ratio << "   "<<a_ratio);
    sample_num ++;
    ////

    return true;
}



bool BigbufDetector::locate_target(cv::Mat &im) {


    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarcy;

    cv::findContours(im, contours,hierarcy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    frame_info target;

    for(int ind = 0; ind < contours.size(); ind++){
        if( contour_valid(contours[ind]) ){
            cv::RotatedRect rect = cv::minAreaRect(contours[ind]);
            cv::Moments M = cv::moments(contours[ind]);

            ////// TO DO: find target in the plate
            float cx = M.m10/M.m00;
            float cy = M.m01/M.m00;
            cv::Point2f mass_ctr = cv::Point(cx,cy);

            cv::Vec4f line;
            cv::fitLine(contours[ind],line,cv::DIST_L2,0,0.01,0.01);
            cv::Point2f direc = cv::Point2f(line[0],line[1]);

            cv::Point2f v = rect.center-mass_ctr;
            v = (direc.dot(v)>0)?(direc):(-direc);
            v = v/sqrt(v.dot(v)); //normalize
            float long_side = (rect.size.height>rect.size.width)?(rect.size.height):(rect.size.width);
            cv::Point2f c_R = mass_ctr + (long_side*13/14)*v;
            cv::Point2f a_vector = mass_ctr-(long_side*1/5)*v;

            cv::circle(im,c_R,4,cv::Scalar(255,255,255),2);
            cv::circle(im,a_vector,1,cv::Scalar(255,255,255),1);

            if(hierarcy[ind][2] == -1) {
                std::cout<< "None"<<std::endl;
                return false;
            }
            cv::RotatedRect inner_rect = cv::minAreaRect(contours[hierarcy[ind][2]]);
            cv::ellipse(this->RGBim,inner_rect,cv::Scalar(0,0,255),3);
            SHOW_IM("RGB",this->RGBim);
            cv::Point2f rect_points[4];
            inner_rect.points(rect_points);
            std::vector<cv::Point2f> armor_points;
            for(int n =0; n<4;n++) armor_points.push_back(rect_points[n]);


            target.center= c_R;
            target.angular_vector = a_vector;
            target.f_time = clock();
            target.armor_points = armor_points;
            record_info(target);
            return true;
        }
    }
}

void BigbufDetector::record_info(frame_info frameInfo) {

    this->frames.insert(frames.begin(), frameInfo);
    refresh_frames();
}

void BigbufDetector::feed_im(cv::Mat& input_image) {

    // check
//    if(  input_image.cols != this->IMAGE_COLS || input_image.rows != this->IMAGE_ROWS)
//          return;

    this->image = input_image;

    filte_image(this->image);

    if(locate_target(this->image)&& this->frames.size() >=3){
        make_prediction();
    }
    cv::imshow("bigbuff",this->image);
    cv::waitKey(1);
}

void BigbufDetector::getTest_result() {

    if(!this->image.rows==0)
        SHOW_IM("Test result",this->image);
}

void BigbufDetector::make_prediction() {
    cv::Mat rvec,tvec;
    cv::Point3f target_3d;
    std::vector<cv::Point2f> armor_points = this->frames[0].armor_points;
    cv::solvePnP(this->real_armor_points, armor_points, cameraMatrix, distCoeffs, rvec, tvec);
    target_3d = cv::Point3f(tvec);
    int pitch = int((atan2(target_3d.y - 80, target_3d.z)+ (float) (OFFSET_PITCH * CV_PI / 1800))*0.6 * 10000);
    //int pitch = 15000;
    int yaw = int((-atan2(target_3d.x, target_3d.z) + (float) (OFFSET_PITCH * CV_PI / 1800)) *0.6* 10000);
    DISP("yaw"<<yaw);
    DISP("pitch"<<pitch);
    serial_gimbal_data data;
    data.size = 6;
    data.rawData[0]= data.head;
    data.rawData[1]= data.id;
    data.rawData[2] = pitch;
    data.rawData[3] = pitch >> 8;
    data.rawData[4] = yaw;
    data.rawData[5] = yaw >> 8;

    this->SP.send_data(data);
}

BigbufDetector::~BigbufDetector() {

}
