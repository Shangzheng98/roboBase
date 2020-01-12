#include "referee_system.h"
namespace roborts_base {
	RefereeSystem::RefereeSystem(std::shared_ptr<roborts_sdk::Handle> handle)
		: handle_(handle) {
		SDK_Init();
	}
	void RefereeSystem::SDK_Init() {
		handle_->CreateSubscriber<roborts_sdk::cmd_game_state>(REFEREE_GAME_CMD_SET, CMD_GAME_STATUS,
			CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
			std::bind(&RefereeSystem::GameStateCallback,
				this,
				std::placeholders::_1));

		handle_->CreateSubscriber<roborts_sdk::cmd_game_robot_state>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_STATUS,
			CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
			std::bind(&RefereeSystem::RobotStatusCallback,
				this,
				std::placeholders::_1));
		handle_->CreateSubscriber<roborts_sdk::cmd_power_heat_data>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_POWER_HEAT,
			CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
			std::bind(&RefereeSystem::RobotHeatCallback,
				this,
				std::placeholders::_1));

		handle_->CreateSubscriber<roborts_sdk::cmd_robot_hurt>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_HURT,
			CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
			std::bind(&RefereeSystem::RobotDamageCallback,
				this,
				std::placeholders::_1));
		handle_->CreateSubscriber<roborts_sdk::cmd_shoot_data>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_SHOOT,
			CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
			std::bind(&RefereeSystem::RobotShootCallback,
				this,
				std::placeholders::_1));
	}

	void RefereeSystem::GameStateCallback(const std::shared_ptr<roborts_sdk::cmd_game_state> raw_game_status) {
		GameStatus game_status;
		game_status.game_status = raw_game_status->game_progress;
		game_status.remaining_time = raw_game_status->stage_remain_time;
	}

	void RefereeSystem::RobotStatusCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_state> raw_robot_status) {
		RobotStatus robot_status;

		if (robot_id_ != raw_robot_status->robot_id) {
			robot_status.id = raw_robot_status->robot_id;
		}
		robot_id_ = raw_robot_status->robot_id;
		//("regeree system id %d",robot_id_);
		robot_status.level = raw_robot_status->robot_level;
		robot_status.remain_hp = raw_robot_status->remain_HP;
		robot_status.max_hp = raw_robot_status->max_HP;
		robot_status.heat_cooling_limit = raw_robot_status->shooter_heat0_cooling_limit;
		robot_status.heat_cooling_rate = raw_robot_status->shooter_heat0_cooling_rate;
		robot_status.chassis_output = raw_robot_status->mains_power_chassis_output;
		robot_status.gimbal_output = raw_robot_status->mains_power_gimbal_output;
		robot_status.shooter_output = raw_robot_status->mains_power_shooter_output;
	}

	void RefereeSystem::RobotHeatCallback(const std::shared_ptr<roborts_sdk::cmd_power_heat_data> raw_robot_heat) {
		RobotHeat robot_heat;
		robot_heat.chassis_volt = raw_robot_heat->chassis_volt;
		robot_heat.chassis_current = raw_robot_heat->chassis_current;
		robot_heat.chassis_power = raw_robot_heat->chassis_power;
		robot_heat.chassis_power_buffer = raw_robot_heat->chassis_power_buffer;
		robot_heat.shooter_heat = raw_robot_heat->shooter_heat0;
	}

	void RefereeSystem::RobotDamageCallback(const std::shared_ptr<roborts_sdk::cmd_robot_hurt> raw_robot_damage) {
		RobotDamage robot_damage;
		robot_damage.damage_type = raw_robot_damage->hurt_type;
		robot_damage.damage_source = raw_robot_damage->armor_id;
	}

	void RefereeSystem::RobotShootCallback(const std::shared_ptr<roborts_sdk::cmd_shoot_data> raw_robot_shoot) {
		RobotShoot robot_shoot;
		robot_shoot.frequency = raw_robot_shoot->bullet_freq;
		robot_shoot.speed = raw_robot_shoot->bullet_speed;
	}

}
