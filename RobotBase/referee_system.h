#ifndef ROBORTS_BASE_REFEREE_SYSTEM_H
#define ROBORTS_BASE_REFEREE_SYSTEM_H

#include "roborts_sdk/sdk.h"
#include "msg.h"

namespace roborts_base {
	/**
	 * @brief ROS API for referee system module
	 */
	class RefereeSystem {
	public:
		/**
		 * @brief Constructor of referee system including initialization of sdk and ROS
		 * @param handle handler of sdk
		 */
		RefereeSystem(std::shared_ptr<roborts_sdk::Handle> handle);
		/**
	   * @brief Destructor of referee system
	   */
		~RefereeSystem() = default;
	private:
		/**
		 * @brief Initialization of sdk
		 */
		void SDK_Init();

		/**  Game Related  **/
		void GameStateCallback(const std::shared_ptr<roborts_sdk::cmd_game_state> raw_game_status);

		/**  Robot Related  **/
		void RobotStatusCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_state> raw_robot_status);

		void RobotHeatCallback(const std::shared_ptr<roborts_sdk::cmd_power_heat_data> raw_robot_heat);

		void RobotDamageCallback(const std::shared_ptr<roborts_sdk::cmd_robot_hurt> raw_robot_damage);

		void RobotShootCallback(const std::shared_ptr<roborts_sdk::cmd_shoot_data> raw_robot_shoot);

		//! sdk handler
		std::shared_ptr<roborts_sdk::Handle> handle_;

		std::shared_ptr<roborts_sdk::Publisher<roborts_sdk::cmd_supply_projectile_booking>>     projectile_supply_pub_;

		uint8_t robot_id_ = 0xFF;
	};
}

#endif //ROBORTS_BASE_REFEREE_SYSTEM_H
