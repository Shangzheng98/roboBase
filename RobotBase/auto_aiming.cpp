#include "gimbal.h"
#include "auto_aiming.h"
#include "armor_detection/armor_detection_node.h"


/*
 * init the realsense camera
 */
int auto_aiming_init(rs2::pipeline_profile pipe_profile)
{
	pipe_profile.get_device().query_sensors()[0].set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
	pipe_profile.get_device().query_sensors()[0].set_option(RS2_OPTION_LASER_POWER, 360);

        //if (ENEMY_COLOR == BLUE)
        if (id >=1 && id <= 7)
	{
		pipe_profile.get_device().query_sensors()[1].set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
		pipe_profile.get_device().query_sensors()[1].set_option(RS2_OPTION_EXPOSURE, 5);
                printf("enemy color is blue");
	}
        else if (id <=17 && id >= 11)
	{
		pipe_profile.get_device().query_sensors()[1].set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
		pipe_profile.get_device().query_sensors()[1].set_option(RS2_OPTION_EXPOSURE, 10);
                printf("enemy color is red");
	}
	//gimbal->CtrlFricWheel(false);
}

/**
 * convert frame from realsense to opencv Mat format
 * @param frames
 * @param align_to_color
 * @param hole_filling_filter
 * @param armor_detection
 * @return
 */
int auto_aiming(rs2::frameset& frames, rs2::align& align_to_color,
	rs2::hole_filling_filter& hole_filling_filter, ArmorDetectionNode& armor_detection)
{
	//将深度图和色彩图对齐
	frames = align_to_color.process(frames);

	rs2::frame color_frame = frames.get_color_frame();
	rs2::frame depth_frame = frames.get_depth_frame();

	if (RES_HOLE_FILLING)
		depth_frame = hole_filling_filter.process(depth_frame);

#if !CONNECT_TO_SERIAL
	if (DEBUG_ENABLE) {
		static rs2::colorizer color_map;
		rs2::frame color_depth_frame = color_map.process(depth_frame);
		cv::Mat color_depth(cv::Size(RE2_FRAME_WIDTH, RE2_FRAME_HEIGHT), CV_8UC3, (void*)color_depth_frame.get_data(), cv::Mat::AUTO_STEP);
		cv::imshow("color_depth", color_depth);
		cv::waitKey(1);
	}
#endif

	cv::Mat frame(cv::Size(RE2_FRAME_WIDTH, RE2_FRAME_HEIGHT), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
	cv::Mat depth(cv::Size(RE2_FRAME_WIDTH, RE2_FRAME_HEIGHT), CV_16U, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

	//将图像传给识别模块
	// pass image to detection moudule
	armor_detection.cv_toolbox_->ImageCallback(frame, depth);

	//识别装甲
	//因为不再使用多线程,这里实际上仅会执行一次
	//如果在nano上帧数低可以考虑重新用回多线程
	armor_detection.ExecuteLoop();

	return 0;
}
