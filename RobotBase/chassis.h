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

#ifndef ROBORTS_BASE_CHASSIS_H
#define ROBORTS_BASE_CHASSIS_H
#include "roborts_sdk/sdk.h"
#include "msg.h"

class Chassis {
public:
	Chassis(std::shared_ptr<roborts_sdk::Handle> handle);
	~Chassis();
	void SDK_Init();
	void ChassisInfoCallback(const std::shared_ptr<roborts_sdk::cmd_chassis_info> chassis_info);
	void CtrlChassisSpeed(Twist vel);
	void CtrlChassisSpeedAcc(TwistAccel vel_acc);
	std::shared_ptr<roborts_sdk::Handle> handle_;
	std::shared_ptr<roborts_sdk::Client<roborts_sdk::cmd_version_id,
		roborts_sdk::cmd_version_id>> verison_client_;
	std::thread heartbeat_thread_;
	std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_heartbeat>> heartbeat_pub_;
	std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_chassis_speed>> chassis_speed_pub_;
	std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_chassis_spd_acc>> chassis_spd_acc_pub_;
};

extern Chassis* chassis;

#endif
