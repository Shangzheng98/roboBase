#include "big_buff.h"
#include "gimbal.h"

void Erosion(int type, cv::Mat frame, int size);
void Dilation(int type, cv::Mat frame, int size);
cv::Mat processedImg(cv::Mat img);
std::vector<cv::Point> GetBuffHitArea(cv::Mat frame);
cv::Point GetCircleCenter(cv::Mat frame, cv::Point buff_point);

int redLowH1 = 0;
int redHighH1 =68 ;
int redLowH2 = 0;
int redHighH2 = 3;

int blueLowH = 90;
int blueHighH = 110;

int LowS = 100;
int HighS = 255;

int LowV = 100;
int HighV = 255;

int dilation_size = 3;
int erosion_size = 1;

int center_size_min = 0;
int center_size_max = 40;

int circle_radius_min = 30;
int circle_radius_max = 50;

int estimate_angle = 45;
int estimate_height = 20;

int imshow_on = 0;

int big_buff_init(rs2::pipeline_profile pipe_profile)
{
	pipe_profile.get_device().query_sensors()[1].set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
        pipe_profile.get_device().query_sensors()[1].set_option(RS2_OPTION_EXPOSURE, 9);
        pipe_profile.get_device().query_sensors()[1].set_option(RS2_OPTION_BRIGHTNESS, 0);
        pipe_profile.get_device().query_sensors()[1].set_option(RS2_OPTION_GAIN,0);
	//gimbal->CtrlFricWheel(true);
        printf("big buff start");
}

int big_buff(rs2::frameset frames)
{
	rs2::frame color_frame = frames.get_color_frame();
	cv::Mat frame(cv::Size(RE2_FRAME_WIDTH, RE2_FRAME_HEIGHT), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        //cvtColor(frame,frame,cv::COLOR_BGR2RGB);
        cv::Mat bin_frame = processedImg(frame);
	//获取大幅的射击点
	std::vector<cv::Point> buff_area = GetBuffHitArea(bin_frame);
	if (buff_area.empty())
	{

		if (DEBUG_ENABLE)
		{
			imshow("video", frame);
			cv::waitKey(1);
		}
		return 0;
	}

	cv::RotatedRect buff_hit_rect = minAreaRect(buff_area);
	cv::Point2f buff_hit_point = buff_hit_rect.center;
	//获取大幅的中心点
	cv::Point circle_center = GetCircleCenter(bin_frame, buff_hit_point);


        // get direction
        static std::chrono::steady_clock::time_point last_calc = {};
        std::chrono::steady_clock::time_point current_calc = std::chrono::steady_clock::now();
        float angle = -atan2(buff_hit_point.y - circle_center.y, buff_hit_point.x-circle_center.x)/3.1415926*180;
        static float last_angle = - 360;
        static int spin_direction = 0;//0=unknown, 1 = clockwise, 2 = counter_clockwise
        if (last_angle == -360)
        {
            last_calc = current_calc;
            last_angle = angle;
            spin_direction = 0;
        }
        else
        {

            if (std::chrono::duration_cast<std::chrono::duration<double>>(current_calc-last_calc).count() > 0.1)
            {
                last_calc = current_calc;
                float angle_diff = angle-last_angle;
                last_angle = angle;
                if(abs(angle_diff)<90)
                {
                    if(angle_diff>0)
                    {
                        spin_direction = 2;
                    }
                    else if(angle_diff<0)
                    {
                        spin_direction = 1;

                    }
                    else
                    {
                        spin_direction = 0;
                    }
                }
            }
        }

	//没识别到大幅
	if (circle_center.x == 0 && circle_center.y == 0)
	{

		if (DEBUG_ENABLE)
		{
			imshow("video", frame);
			cv::waitKey(1);
		}
		return 0;
	}

	//将击打点绕中心旋转estimate_angle度
        bool can_estimate = false;
//        printf("remain_time: %d\n", remain_time);
//        if(remain_time< 360 && remain_time>=240)
//        {
//            can_estimate =false;
//        }
//        else
//        {
//            can_estimate = true;
//        }

        if (true)
        {

            cv::Point2f estimate_hit_point = {};
            if (spin_direction ==2) // counter_clockwise
            {
                estimate_hit_point.x = (buff_hit_point.x - circle_center.x) * cos(3.14 / 180.0 * -estimate_angle) -
                     (buff_hit_point.y - circle_center.y) * sin(3.14 / 180.0 * -estimate_angle) + circle_center.x;
                estimate_hit_point.y = (buff_hit_point.x - circle_center.x) * sin(3.14 / 180.0 * -estimate_angle) +
                     (buff_hit_point.y - circle_center.y) * cos(3.14 / 180.0 * -estimate_angle) + circle_center.y;
            }
            else
            {
                estimate_hit_point.x = (buff_hit_point.x - circle_center.x) * cos(3.14 / 180.0 * estimate_angle) -
                     (buff_hit_point.y - circle_center.y) * sin(3.14 / 180.0 * estimate_angle) + circle_center.x;
                estimate_hit_point.y = (buff_hit_point.x - circle_center.x) * sin(3.14 / 180.0 * estimate_angle) +
                     (buff_hit_point.y - circle_center.y) * cos(3.14 / 180.0 * estimate_angle) + circle_center.y;
            }


            estimate_hit_point.y -= estimate_height;
         //深度7, 不需要重力补偿,不需要针对装甲板的预测
         gimbal->AimAtArmor(estimate_hit_point.x, estimate_hit_point.y, cv::Mat(), false, false, true);
         //下面的代码是用来画辅助线的
         if (DEBUG_ENABLE)
         {
            cv::Size2f estimate_hit_area_size = buff_hit_rect.size;
            estimate_hit_area_size.height *= 1.5;
            estimate_hit_area_size.width *= 1.5;

            cv::RotatedRect estimate_hit_area(estimate_hit_point, estimate_hit_area_size, buff_hit_rect.angle + estimate_angle);
            line(frame, circle_center, buff_hit_point, cv::Scalar(255, 0, 0), 2);
            line(frame, circle_center, estimate_hit_point, cv::Scalar(0, 0, 255), 2);
            cv::Point2f vertices[4] = { };
            buff_hit_rect.points(vertices);
            std::vector<cv::Point> contour;
            for (int i = 0; i < 4; i++)
            {
                 contour.push_back(vertices[i]);
            }
            std::vector<std::vector<cv::Point>> contours;
            contours.push_back(contour);
            drawContours(frame, contours, 0, cv::Scalar(255, 255, 0), 2);

             contour.clear();
             contours.clear();
             estimate_hit_area.points(vertices);
             for (int i = 0; i < 4; i++)
             {
                 contour.push_back(vertices[i]);
             }
             contours.push_back(contour);
             drawContours(frame, contours, 0, cv::Scalar(255, 255, 0), 2);

             int distance = sqrtf(powf((circle_center.x - buff_hit_point.x), 2) + powf((circle_center.y - buff_hit_point.y), 2));
             if (distance > 10)
             {
                 circle(frame, circle_center, distance + distance * 0.15, cv::Scalar(0, 255, 0));
                 circle(frame, circle_center, distance - distance * 0.15, cv::Scalar(0, 255, 0));
             }
             imshow("video", frame);
             cv::waitKey(1);
         }
        }
        else
        {

            buff_hit_point.y -= estimate_height;
            gimbal -> AimAtArmor(buff_hit_point.x, buff_hit_point.y,cv::Mat(),false,false,true);


            cv::Point2f vertices[4] = { };
            buff_hit_rect.points(vertices);
            std::vector<cv::Point> contour;
            for (int i = 0; i < 4; i++)
            {
                 contour.push_back(vertices[i]);
            }
            std::vector<std::vector<cv::Point>> contours;
            contours.push_back(contour);
            drawContours(frame, contours, 0, cv::Scalar(255, 255, 0), 2);
            contour.clear();
            contours.clear();
            imshow("video", frame);
            cv::waitKey(1);
        }

	return 0;
}


cv::Point GetCircleCenter(cv::Mat frame, cv::Point buff_point)
{
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	findContours(frame, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	int smallest_index = -1;
	for (int i = 0; i < contours.size(); i++)
	{

		cv::Point2f center;
		float radius;
		minEnclosingCircle(contours[i], center, radius);
		int distance = sqrtf(powf((center.x - buff_point.x), 2) + powf((center.y - buff_point.y), 2));
		if (contours[i].size() < center_size_min || contours[i].size() > center_size_max)
			continue;
		if (distance < circle_radius_min || distance > circle_radius_max)
			continue;

		if (smallest_index == -1)
			smallest_index = i;
		else if (contours[smallest_index].size() > contours[i].size())
			smallest_index = i;

	}
	if (smallest_index == -1)
		return cv::Point();
	cv::Point2f center;
	float radius;
	minEnclosingCircle(contours[smallest_index], center, radius);
	return center;
}

std::vector<cv::Point> GetBuffHitArea(cv::Mat frame)
{
	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;
	findContours(frame, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	cv::Scalar color(0, 255, 0);
	for (int index = 0; index < contours.size(); index++) {
		if (hierarchy[index][0] != -1 || hierarchy[index][1] != -1) {
			continue;
		}
		return contours[index];
	}
	return std::vector<cv::Point>();
}

void Erosion(int type, cv::Mat frame, int size)
{
	if (size == 0)
		return;
	switch (type)
	{
	case 0:
		type = cv::MORPH_RECT;
		break;
	case 1:
		type = cv::MORPH_CROSS;
		break;
	case 2:
		type = cv::MORPH_ELLIPSE;
		break;
	default:
		type = cv::MORPH_ELLIPSE;
		break;
	}
	cv::Mat element = cv::getStructuringElement(type, cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));
	erode(frame, frame, element);
}

void Dilation(int type, cv::Mat frame, int size)
{
	if (size == 0)
		return;
	switch (type)
	{
	case 0:
		type = cv::MORPH_RECT;
		break;
	case 1:
		type = cv::MORPH_CROSS;
		break;
	case 2:
		type = cv::MORPH_ELLIPSE;
		break;
	default:
		type = cv::MORPH_ELLIPSE;
		break;
	}
	cv::Mat element = cv::getStructuringElement(type, cv::Size(2 * size + 1, 2 * size + 1), cv::Point(size, size));
	dilate(frame, frame, element);
}
cv::Mat processedImg(cv::Mat img)
{
	cv::Mat HSVimg;
//#if OPENCV_CUDA_ENABLE
//	cv::cuda::GpuMat src_gpu, dst_gpu;
//	src_gpu.upload(img);
 //       cv::cuda::cvtColor(src_gpu, dst_gpu, cv::COLOR_BGR2GRAY);
//	dst_gpu.download(HSVimg);
//#else
        cvtColor(img, HSVimg, cv::COLOR_BGR2HSV);
	//cvtColor(img, HSVimg, cv::COLOR_BGR2HSV);
//#endif
	cv::Mat result;
        //if ( ENEMY_COLOR== BLUE) //we are red
        printf("big buff id %d\n",id);
        if(id <=7 && id>=1)
	{
		cv::Mat result1, result2;
		inRange(HSVimg, cv::Scalar(redLowH1, LowS, LowV), cv::Scalar(redHighH1, HighS, HighV), result);
		//inRange(HSVimg, cv::Scalar(redLowH2, LowS, LowV), cv::Scalar(redHighH2, HighS, HighV), result2);
		//result = result1 | result2;
	}
	else
	{
		inRange(HSVimg, cv::Scalar(blueLowH, LowS, LowV), cv::Scalar(blueHighH, HighS, HighV), result);
	}
	//GaussianBlur(result, result, Size(5 * 2 + 1, 5 * 2 + 1), 0, 0, 4);
	//Canny(result, result, 3, 9, 3);
	//Erosion(2, result, 1);
	Dilation(2, result, dilation_size);
	Erosion(2, result, erosion_size);
	if (DEBUG_ENABLE)
	{
            imshow("re", result);
	}
	return result;
}
