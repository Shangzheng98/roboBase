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

    cv::GaussianBlur(im,im,cv::Size(3,3),0,0);
    cv::Mat ref_im;
    inRange(im, cv::Scalar(THR_B_min, THR_G_min, THR_R_min), cv::Scalar(THR_B_max, THR_G_max, THR_R_max), im);
    //inRange(im,cv::Scalar(0,THR_S_min,THR_V_min),cv::Scalar(360,THR_S_max,THR_V_max),im);

    //im = im & ref_im;
    //morphologyEx(im, im, 3, getStructuringElement(0, cv::Size(3, 3), cv::Point(2, 2)));
    //cv::dilate(im,im,cv::Mat());
    cv::erode(im,im,cv::Mat());
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
    if(area<2500)
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

    if( 2> l_ratio || l_ratio >2.17)
        return false;


    float a_ratio = rect.size.area()/area;
    ///test///std::cout<< "AREA RATIO:  "<<a_ratio<<std::endl;

    /// TO DO: set threshold
    if(a_ratio > 2.6 || a_ratio <2)
        return false;
    ///test:
    drawContour_Rec(rect, image);
    std::cout<< "AREA:  "<<area<<std::endl;
    return true;
}



cv::Point2f BigbufDetection::locate_target(cv::Mat &im) {


    std::vector<std::vector<cv::Point>> contours;

    cv::findContours(im, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    for(int ind = 0; ind < contours.size(); ind++){
        if( contour_valid(contours[ind]) ){
            cv::RotatedRect rect = cv::minAreaRect(contours[ind]);
            cv::Moments M = cv::moments(contours,true);
            ////// TO DO: find target in the plate
            float cx = M.m10/M.m00;
            float cy = M.m01/M.m00;

            if(test)
                cv::circle(im,rect.center,4,cv::Scalar(0,0,0));
            return rect.center;
        }
    }
    return cv::Point2f(0,0);


}

void BigbufDetection::record_info(clock_t t, cv::Point2f& pos) {
    if (pos.x==0 && pos.y==0)
        return;
    struct frame_info frameInfo;
    frameInfo.f_time = t;
    frameInfo.position = pos;
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
        cv::circle(this->image,frames[0].position,4,cv::Scalar(233,233,233),3);
    if(!this->image.empty());
        cv::imshow("Test result",this->image);

}

void BigbufDetection::make_prediction() {

}
