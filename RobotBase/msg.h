#ifndef MSG_H
#define MSG_H

#include <stdlib.h>

struct RobotDamage
{
	uint8_t ARMOR = 0;
	uint8_t OFFLINE = 1;
	uint8_t EXCEED_HEAT = 2;
	uint8_t EXCEED_POWER = 3;
	uint8_t damage_type;
	uint8_t FORWARD = 0;
	uint8_t LEFT = 1;
	uint8_t BACKWARD = 2;
	uint8_t RIGHT = 3;
	uint8_t damage_source;
};

struct RobotHeat
{
	uint16_t chassis_volt;
	uint16_t chassis_current;
	float  chassis_power;
	uint16_t chassis_power_buffer;
	uint16_t shooter_heat;
};

struct RobotShoot
{
	uint8_t frequency;
	float speed;
};

struct RobotStatus
{
	uint8_t id;
	uint8_t level;
	uint16_t remain_hp;
	uint16_t max_hp;
	uint16_t heat_cooling_limit;
	uint16_t heat_cooling_rate;
	bool gimbal_output;
	bool chassis_output;
	bool shooter_output;
};

struct GameStatus
{
	uint8_t PRE_MATCH = 0;
	uint8_t SETUP = 1;
	uint8_t INIT = 2;
	uint8_t FIVE_SEC_CD = 3;
	uint8_t ROUND = 4;
	uint8_t CALCULATION = 5;
	uint8_t game_status;
	uint16_t remaining_time;
};

struct GimbalAngle
{
	bool yaw_mode;
	bool pitch_mode;
	float yaw_angle;
	float pitch_angle;
};

struct GimbalRate
{
	float pitch_rate;
	float yaw_rate;
};

struct ShootInfo
{
	int16_t remain_bullet;
	int16_t sent_bullet;
	bool fric_wheel_run;
};

struct ShootState
{
	int32_t single_shoot;
	int32_t continue_shoot;
	int32_t run_friction_whell;
	int32_t friction_whell_speed;
};

struct Twist
{
	struct Vector3
	{
		float x;
		float y;
		float z;
	};
	Vector3  linear;
	Vector3  angular;
};

struct TwistAccel
{
	Twist twist;
	Twist accel;
};


#endif
