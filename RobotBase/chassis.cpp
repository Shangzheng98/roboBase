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

#include "chassis.h"

#define M_PI 3.14159265358979323846;

Chassis::Chassis(std::shared_ptr<roborts_sdk::Handle> handle)
	: handle_(handle) {
	SDK_Init();
}
Chassis::~Chassis() {
	if (heartbeat_thread_.joinable()) {
		heartbeat_thread_.join();
	}
}
void Chassis::SDK_Init() {

	verison_client_ = handle_->CreateClient<roborts_sdk::cmd_version_id, roborts_sdk::cmd_version_id>
		(UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
			MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
	roborts_sdk::cmd_version_id version_cmd;
	version_cmd.version_id = 0;
	auto version = std::make_shared<roborts_sdk::cmd_version_id>(version_cmd);
	verison_client_->AsyncSendRequest(version,
		[](roborts_sdk::Client<roborts_sdk::cmd_version_id,
			roborts_sdk::cmd_version_id>::SharedFuture future) {
				printf("Chassis Firmware Version: %d.%d.%d.%d\n",
					int(future.get()->version_id >> 24 & 0xFF),
					int(future.get()->version_id >> 16 & 0xFF),
					int(future.get()->version_id >> 8 & 0xFF),
					int(future.get()->version_id & 0xFF));
		});

	handle_->CreateSubscriber<roborts_sdk::cmd_chassis_info>(CHASSIS_CMD_SET, CMD_PUSH_CHASSIS_INFO,
		CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
		std::bind(&Chassis::ChassisInfoCallback, this, std::placeholders::_1));

	chassis_speed_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_speed>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPEED,
		MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
	chassis_spd_acc_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_chassis_spd_acc>(CHASSIS_CMD_SET, CMD_SET_CHASSIS_SPD_ACC,
		MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);

	heartbeat_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_heartbeat>(UNIVERSAL_CMD_SET, CMD_HEARTBEAT,
		MANIFOLD2_ADDRESS, CHASSIS_ADDRESS);
	heartbeat_thread_ = std::thread([this] {
		roborts_sdk::cmd_heartbeat heartbeat;
		heartbeat.heartbeat = 0;
		while (true) {
			heartbeat_pub_->Publish(heartbeat);
			std::this_thread::sleep_for(std::chrono::milliseconds(300));
		}
		}
	);


}

void Chassis::ChassisInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_info> chassis_info)
{

	/*
	ros::Time current_time = ros::Time::now();
	odom_.header.stamp = current_time;
	odom_.pose.pose.position.x = chassis_info->position_x_mm / 1000.;
	odom_.pose.pose.position.y = chassis_info->position_y_mm / 1000.;
	odom_.pose.pose.position.z = 0.0;
	geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(chassis_info->gyro_angle / 1800.0 * M_PI);
	odom_.pose.pose.orientation = q;
	odom_.twist.twist.linear.x = chassis_info->v_x_mm / 1000.0;
	odom_.twist.twist.linear.y = chassis_info->v_y_mm / 1000.0;
	odom_.twist.twist.angular.z = chassis_info->gyro_rate / 1800.0 * M_PI;
	ros_odom_pub_.publish(odom_);

	odom_tf_.header.stamp = current_time;
	odom_tf_.transform.translation.x = chassis_info->position_x_mm / 1000.;
	odom_tf_.transform.translation.y = chassis_info->position_y_mm / 1000.;

	odom_tf_.transform.translation.z = 0.0;
	odom_tf_.transform.rotation = q;
	tf_broadcaster_.sendTransform(odom_tf_);*/

	//TODO

}

void Chassis::CtrlChassisSpeed(Twist vel) {
	roborts_sdk::cmd_chassis_speed chassis_speed;
	chassis_speed.vx = vel.linear.x * 1000;
	chassis_speed.vy = vel.linear.y * 1000;
	chassis_speed.vw = vel.angular.z * 1800.0 / M_PI;
	chassis_speed.rotate_x_offset = 0;
	chassis_speed.rotate_y_offset = 0;
	chassis_speed_pub_->Publish(chassis_speed);
}

void Chassis::CtrlChassisSpeedAcc(TwistAccel vel_acc) {
	roborts_sdk::cmd_chassis_spd_acc chassis_spd_acc;
	chassis_spd_acc.vx = vel_acc.twist.linear.x * 1000;
	chassis_spd_acc.vy = vel_acc.twist.linear.y * 1000;
	chassis_spd_acc.vw = vel_acc.twist.angular.z * 1800.0 / M_PI;
	chassis_spd_acc.ax = vel_acc.accel.linear.x * 1000;
	chassis_spd_acc.ay = vel_acc.accel.linear.y * 1000;
	chassis_spd_acc.wz = vel_acc.accel.angular.z * 1800.0 / M_PI;
	chassis_spd_acc.rotate_x_offset = 0;
	chassis_spd_acc.rotate_y_offset = 0;
	chassis_spd_acc_pub_->Publish(chassis_spd_acc);
}