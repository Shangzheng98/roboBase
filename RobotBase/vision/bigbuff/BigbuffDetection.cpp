//
// Created by eric on 2/15/20.
//

#include "BigbuffDetection.h"


BigbufDetection::BigbufDetection(int cols, int rows, bool test){
    this->IMAGE_COLS = cols;
    this->IMAGE_ROWS = rows;
    this->test = test;
}

void BigbufDetection::refresh_frames() {

    // clear the memory
    if(this->frames.size() > 3){
        frames.erase(frames.end());
    }

    // if the time interval between first two consecutive frames is to large
    // then erase all except for the first one.
    if(this->frames.size()>2 && (this->frames[0].f_time-this->frames[1].f_time) > 0.1 ){
        frames.erase(frames.begin()+1,frames.end());
    }
    ///// TO DO: refresh when angle between two frames are too large.
}

void BigbufDetection::filte_image(cv::Mat &im) {

    //cv::GaussianBlur(im,im,cv::Size(3,3),0,0);
    cv::Mat binary_brightness_img, binary_color_img, gray;
    cv::Mat ref_im;
    //inRange(im, cv::Scalar(THR_B_min, THR_G_min, THR_R_min), cv::Scalar(THR_B_max, THR_G_max, THR_R_max), im);
    cvtColor(im, gray, cv::COLOR_BGR2GRAY);
    cv::Mat result_img;
    std::vector<cv::Mat> BGR_channels;
    split(im, BGR_channels);
    ///test:
    cv::imshow("Color",im);
    if (this->color_ == 0) // opposite red
    {
        subtract(BGR_channels[2], BGR_channels[1],result_img);
    } else {
        subtract(BGR_channels[0], BGR_channels[2], result_img);
    }
    threshold(gray, binary_brightness_img, gray_th_, 255, cv::THRESH_BINARY);
    threshold(result_img, binary_color_img, color_th_, 255, cv::THRESH_BINARY);
    ///test:
    imshow("binary_brightness_img", binary_brightness_img);
    imshow("binary_color_img", binary_color_img);
    im = binary_color_img & binary_brightness_img;
    //im = im & ref_im;
    //morphologyEx(im, im, 3, getStructuringElement(0, cv::Size(3, 3), cv::Point(2, 2)));

    cv::dilate(binary_color_img,binary_color_img,getStructuringElement(0, cv::Size(5, 5)));

    cv::erode(binary_color_img,binary_color_img,getStructuringElement(0, cv::Size(5, 5)));
}


void BigbufDetection::drawContour_Rec(cv::RotatedRect rect, cv::Mat &src) {

    cv::Point2f *vertices = new cv::Point2f[4];

    rect.points(vertices);

    for (int j = 0; j < 4; j++)
    cv::line(src, vertices[j] , vertices[(j + 1) % 4] , cv::Scalar(180, 180, 180), 1);

}

bool  BigbufDetection::contour_valid(std::vector<cv::Point>& contour){
    ///// TO DO: find suitable threshold for area
    float area = cv::contourArea(contour);
    if(area<100)
        return false;

    cv::RotatedRect rect = cv::minAreaRect(contour);
    ///// notice that l_ratio is always bigger than 0
    float l_ratio;
    if(rect.size.height > rect.size.width){
        l_ratio = rect.size.height/rect.size.width;
    }else{
        l_ratio = rect.size.width/rect.size.height;
    }
    ///test///std::cout<< "LENGTH RATIO:  "<< l_ratio<<std::endl;
    if(l_ratio<1.9 || 2.4< l_ratio)
        return false;


    float a_ratio = rect.size.area()/area;
    ///test///std::cout<< "AREA RATIO:  "<<a_ratio<<std::endl;
    /// TO DO: set threshold
    if(a_ratio < 2 || a_ratio>2.5)
        return false;

    ///test:
    drawContour_Rec(rect, image);
    std::cout<<this->sample_num<< "   "<<l_ratio << "   "<<a_ratio<<std::endl;
    sample_num ++;
    ////

    return true;
}



cv::Point2f BigbufDetection::locate_target(cv::Mat &im) {


    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(im, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    frame_info target;

    for(int ind = 0; ind < contours.size(); ind++){
        cv::approxPolyDP(contours[ind],contours[ind],1,true);
        if( contour_valid(contours[ind]) ){
            cv::RotatedRect rect = cv::minAreaRect(contours[ind]);
            cv::Moments M = cv::moments(contours[ind]);

            ////// TO DO: find target in the plate
            float cx = M.m10/M.m00;
            float cy = M.m01/M.m00;
            cv::Point2f v = rect.center-cv::Point2f(cx,cy);
            v = v/sqrt(v.dot(v)); ///normalize
            int long_side;
            if (rect.size.height > rect.size.width)
                long_side = rect.size.height;
            else
                long_side = rect.size.width;
            cv::Point2f c_R = rect.center + (long_side*11/14)*v;
            cv::Point2f a_vector = (long_side*8/7)*v;
            /// test:
            std::cout<<v.x<<std::endl;
            std::cout<<long_side<<std::endl;
            cv::circle(this->image,c_R,4,cv::Scalar(255,255,255),3);
            target.center = rect.center;
            return rect.center;
        }
    }
    target.center = cv::Point2f(0,0);
    return cv::Point2f(0,0);


}

void BigbufDetection::record_info(clock_t t, cv::Point2f& pos) {
    if (pos.x==0 && pos.y==0) /// exceptional case
        return;

    struct frame_info frameInfo;
    frameInfo.f_time = t;
    frameInfo.center = pos;

    this->frames.insert(frames.begin(), frameInfo);
    refresh_frames();
}

void BigbufDetection::feed_im(cv::Mat& input_image) {

    // check
    if( !this->test && (input_image.cols != this->IMAGE_COLS || input_image.rows != this->IMAGE_ROWS))
        return;

    clock_t t = clock();

    this->image = input_image;

    filte_image(this->image);

    cv::Point2f target = locate_target(this->image);

    //record_info(t,target);

}

void BigbufDetection::getTest_result() {
    if(!frames.empty())
        cv::circle(this->image,frames[0].center,4,cv::Scalar(233,233,233),3);
    if(!this->image.empty());
        cv::imshow("Test result",this->image);

}

void BigbufDetection::make_prediction() {

}
