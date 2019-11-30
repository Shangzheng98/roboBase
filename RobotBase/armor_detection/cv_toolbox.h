/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify *  it under the terms of the GNU General Public License as published by *  the Free Software Foundation, either version 3 of the License, or *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_DETECTION_CVTOOLBOX_H
#define ROBORTS_DETECTION_CVTOOLBOX_H

#include <vector>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>
#include "../gimbal.h"
#include <pthread.h>

extern int redLowH1;
extern int redHighH1;
extern int redLowH2;
extern int redHighH2;

extern int blueLowH;
extern int blueHighH;

extern int LowS;
extern int HighS;

extern int LowV;
extern int HighV;

/**
 * This class is a toolbox for RoboMaster detection.
 */

enum BufferState {
	IDLE = 0,
	WRITE,
	READ
};

class CVToolbox {
public:
	/**
	 *  @brief The constructor of the CVToolbox.
	 */
	explicit CVToolbox(std::string camera_name,
		unsigned int buffer_size = 1) :
		get_img_info_(false)
	{
		image_buffer_.resize(buffer_size);
		depth_buffer_.resize(buffer_size);
		buffer_state_.resize(buffer_size);
		index_ = 0;
		capture_time_ = -1;
		for (int i = 0; i < buffer_state_.size(); ++i) {
			buffer_state_[i] = BufferState::IDLE;
		}
		latest_index_ = -1;
		pthread_spin_init(&lock_, PTHREAD_PROCESS_PRIVATE);
	}

	void ImageCallback(cv::Mat img, cv::Mat depth) {
		//既然不使用多线程,这里完全没有必要加锁
		pthread_spin_lock(&lock_);
		for (int i = 0; i < buffer_state_.size(); ++i) {
			if (buffer_state_[i] != BufferState::READ) {
				image_buffer_[i] = img.clone();
				depth_buffer_[i] = depth.clone();
				buffer_state_[i] = BufferState::WRITE;
				latest_index_ = i;
			}
		}
		pthread_spin_unlock(&lock_);
	}
	/**
	 * @brief Get next new image.
	 * @param src_img Output image
	 */
	int NextImage(cv::Mat& src_img, cv::Mat& src_depth) {
		if (latest_index_ < 0) {
			printf("Call image when no image received\n");
			return -1;
		}
		int temp_index = -1;
		pthread_spin_lock(&lock_);
		if (buffer_state_[latest_index_] == BufferState::WRITE) {
			buffer_state_[latest_index_] = BufferState::READ;
		}
		else {
			//printf("No image is available\n");
			pthread_spin_unlock(&lock_);
			return temp_index;
		}
		temp_index = latest_index_;
		pthread_spin_unlock(&lock_);

		src_img = image_buffer_[temp_index];
		src_depth = depth_buffer_[temp_index];
		return temp_index;
	}

	void GetCaptureTime(double& capture_time) {
		if (!get_img_info_) {
			//printf("The first image doesn't receive\n");
			return;
		}
		if (capture_time_ < 0) {
			printf("The second image doesn't receive\n");
			return;
		}

		capture_time = capture_time_;
	}

	/**
	 * @brief Return the image after use
	 * @param return_index Index gets from function 'int NextImage(cv::Mat &src_img)'
	 */
	void ReadComplete(int return_index) {

		if (return_index < 0 || return_index >(buffer_state_.size() - 1)) {
			printf("Return index error, please check the return_index\n");
			return;
		}

		buffer_state_[return_index] = BufferState::IDLE;
	}

	/**
	 * @brief Highlight the blue or red region of the image.
	 * @param image Input image ref
	 * @return Single channel image
	 */
	cv::Mat DistillationColor(const cv::Mat& src_img, unsigned int color, bool using_hsv) {
		if (using_hsv)
		{
			cv::Mat img_hsv;
//#if OPENCV_CUDA_ENABLE
//			cv::cuda::GpuMat src_gpu, dst_gpu;
//			src_gpu.upload(src_img);
//			cv::cuda::cvtColor(src_img, dst_gpu, cv::COLOR_BGR2GRAY);
//			dst_gpu.download(img_hsv);
//#else
			cv::cvtColor(src_img, img_hsv, cv::COLOR_BGR2GRAY);
//#endif
			if (color == 0) {
				cv::Mat img_hsv_blue, img_threshold_blue;
				img_hsv_blue = img_hsv.clone();
				cv::Mat blue_low(cv::Scalar(blueLowH, LowS, LowV));
				cv::Mat blue_higher(cv::Scalar(blueHighH, HighS, HighV));
				cv::inRange(img_hsv_blue, blue_low, blue_higher, img_threshold_blue);
				return img_threshold_blue;
			}
			else {
				cv::Mat img_hsv_red1, img_hsv_red2, img_threshold_red, img_threshold_red1, img_threshold_red2;
				img_hsv_red1 = img_hsv.clone();
				img_hsv_red2 = img_hsv.clone();
				cv::Mat red1_low(cv::Scalar(70, 43, 46));
				cv::Mat red1_higher(cv::Scalar(170, 255, 255));

				//cv::Mat red1_low(cv::Scalar(redLowH1, LowS, LowV));
				//cv::Mat red1_higher(cv::Scalar(redHighH1, HighS, HighV));
				cv::inRange(img_hsv_red1, red1_low, red1_higher, img_threshold_red1);
				//cv::inRange(img_hsv_red2, red2_low, red2_higher, img_threshold_red2);
				img_threshold_red = img_threshold_red1; //| img_threshold_red2;
				//cv::imshow("img_threshold_red", img_threshold_red);
				return img_threshold_red;
			}
		}
		else
		{
			std::vector<cv::Mat> bgr;
			cv::split(src_img, bgr);
			if (color == 1) {
				cv::Mat result_img;
				cv::subtract(bgr[2], bgr[1], result_img);
				return result_img;
			}
			else if (color == 0) {
				cv::Mat result_img;
				cv::subtract(bgr[0], bgr[2], result_img);
				return result_img;
			}
		}
	}
	/**
	 * @brief The wrapper for function cv::findContours
	 * @param binary binary image ref
	 * @return Contours that found.
	 */
	std::vector<std::vector<cv::Point>> FindContours(const cv::Mat& binary_img) {
		std::vector<std::vector<cv::Point>> contours;
		const auto mode = cv::RETR_EXTERNAL;
		const auto method = cv::CHAIN_APPROX_SIMPLE;
		cv::findContours(binary_img, contours, mode, method);
		return contours;
	}
	/**
	 * @brief Draw rectangle.
	 * @param img The image will be drew on
	 * @param rect The target rectangle
	 * @param color Rectangle color
	 * @param thickness Thickness of the line
	 */
	void DrawRotatedRect(const cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness) {
		cv::Point2f vertex[4];

		cv::Point2f center = rect.center;
		float angle = rect.angle;
		std::ostringstream ss;
		ss << angle;
		std::string text(ss.str());
		int font_face = cv::FONT_HERSHEY_COMPLEX;
		double font_scale = 0.5;
		cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 255, 255), thickness, 8, 0);

		rect.points(vertex);

		for (int i = 0; i < 4; i++)
			cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);

		//华南虎的展示方式
		//没别的,就是帅
		/*
		cv::line(img, vertex[0], vertex[1], color, thickness);
		cv::line(img, vertex[1], vertex[3], color, thickness);
		cv::line(img, vertex[3], vertex[2], color, thickness);
		cv::line(img, vertex[2], vertex[0], color, thickness);
		*/
	}

	void DrawRotatedRect(const cv::Mat& img, const cv::RotatedRect& rect, const cv::Scalar& color, int thickness, float angle) {
		cv::Point2f vertex[4];

		cv::Point2f center = rect.center;
		std::ostringstream ss;
		ss << (int)(angle);
		std::string text(ss.str());
		int font_face = cv::FONT_HERSHEY_COMPLEX;
		double font_scale = 0.5;
		cv::putText(img, text, center, font_face, font_scale, cv::Scalar(0, 100, 0), thickness, 8, 0);

		rect.points(vertex);
		for (int i = 0; i < 4; i++)
			cv::line(img, vertex[i], vertex[(i + 1) % 4], color, thickness);
	}
private:
	std::vector<cv::Mat> image_buffer_;
	std::vector<cv::Mat> depth_buffer_;
	std::vector<BufferState> buffer_state_;
	int latest_index_;
	pthread_spinlock_t lock_;
	int index_;
	std::chrono::high_resolution_clock::time_point capture_begin_;
	double capture_time_;

	bool get_img_info_;
};

#endif //ROBORTS_DETECTION_CVTOOLBOX_H
