/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#ifndef ROBORTS_BASE_GIMBAL_H
#define ROBORTS_BASE_GIMBAL_H
#include <vector>
#include "roborts_sdk/sdk.h"
#include "msg.h"
#include <opencv2/opencv.hpp>
#include<librealsense2/rsutil.h>
#include <opencv2/core/cuda.hpp>

 //使用CUDA运算,在nano上调为1
#define OPENCV_CUDA_ENABLE 1

extern double OFFSET_X;
extern double OFFSET_Y;
extern double OFFSET_Z;
extern double OFFSET_PITCH;
extern double OFFSET_YAW;

extern int OFFSET_INT_X;
extern int OFFSET_INT_Y;
extern int OFFSET_INT_Z;
extern int OFFSET_INT_PITCH;
extern int OFFSET_INT_YAW;

extern int ARMOR_WIDTH;
extern int ARMOR_HEIGHT;
extern int current_task;
extern int remain_time;
extern int id;
#define BLUE 0
#define RED 1

extern int DEBUG_ENABLE;
extern int ENEMY_COLOR;

extern rs2_intrinsics intrin_rgb;

class Gimbal {
public:
	Gimbal(std::shared_ptr<roborts_sdk::Handle> handle);
	~Gimbal();
	void SDK_Init();
	void GimbalInfoCallback(const std::shared_ptr<roborts_sdk::cmd_gimbal_info> gimbal_info);
	void VisionInfoCallback(const std::shared_ptr<roborts_sdk::cmd_vision_info> vision_info);
        void StageInfoCallback(const std::shared_ptr<roborts_sdk::cmd_game_state> game_info);
        void RobotInfoCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_state> robot_info);
	void CtrlGimbalAngle(GimbalAngle msg);
	bool CtrlGimbalMode(int mode);
	bool CtrlFricWheel(bool open);
	bool CtrlShoot(uint8_t mode, uint8_t number);
	//参数:x,y,深度图,是否重力补偿,是否预判位置,是否射击
	void AimAtArmor(int x, int y, cv::Mat depth_frame, bool compensation, bool prediction, bool shoot);
	std::shared_ptr<roborts_sdk::Handle> handle_;
	std::shared_ptr<roborts_sdk::Client<roborts_sdk::cmd_version_id,
		roborts_sdk::cmd_version_id>> verison_client_;
	std::thread heartbeat_thread_;
	std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_heartbeat>>        heartbeat_pub_;
	std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_gimbal_angle>>     gimbal_angle_pub_;
	std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::gimbal_mode_e>>        gimbal_mode_pub_;
	std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_fric_wheel_speed>> fric_wheel_pub_;
	std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_shoot_info>>       gimbal_shoot_pub_;
private:
	cv::KalmanFilter KF_space;
	cv::KalmanFilter KF_offset;
};

extern Gimbal* gimbal;

#endif
