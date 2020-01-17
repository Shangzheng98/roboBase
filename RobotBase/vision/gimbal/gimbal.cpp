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
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "gimbal.h"

#define M_PI 3.14159265358979323846
#define D2R (M_PI / 180)

#include<librealsense2/rsutil.h>
#include <linux/input.h>
#include <eigen3/Eigen/Core>


//弹速,比真实弹速小2-3可能效果更好
float INIT_V = 20.0f;

pthread_spinlock_t gimbal_to_chassis_lock = {};


Gimbal::Gimbal(std::shared_ptr<roborts_sdk::Handle> handle)
        : handle_(handle) {
    SDK_Init();
    pthread_spin_init(&gimbal_to_chassis_lock, PTHREAD_PROCESS_PRIVATE);
}

Gimbal::~Gimbal() {
    if (heartbeat_thread_.joinable()) {
        heartbeat_thread_.join();
    }
}

void Gimbal::SDK_Init() {

    if (handle_ == NULL)
        return;
    verison_client_ = handle_->CreateClient<roborts_sdk::cmd_version_id, roborts_sdk::cmd_version_id>
            (UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
             MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
    roborts_sdk::cmd_version_id version_cmd;
    version_cmd.version_id = 0;
    auto version = std::make_shared<roborts_sdk::cmd_version_id>(version_cmd);
    verison_client_->AsyncSendRequest(version,
                                      [](roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                              roborts_sdk::cmd_version_id>::SharedFuture future) {
                                          printf("Gimbal Firmware Version: %d.%d.%d.%d\n",
                                                 int(future.get()->version_id >> 24 & 0xFF),
                                                 int(future.get()->version_id >> 16 & 0xFF),
                                                 int(future.get()->version_id >> 8 & 0xFF),
                                                 int(future.get()->version_id & 0xFF));
                                      });
    handle_->CreateSubscriber<roborts_sdk::cmd_game_robot_state>(REFEREE_ROBOT_CMD_SET, CMD_ROBOT_STATUS,
                                                                 CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                                 std::bind(&Gimbal::RobotInfoCallback, this,
                                                                           std::placeholders::_1));

    handle_->CreateSubscriber<roborts_sdk::cmd_gimbal_info>(GIMBAL_CMD_SET, CMD_PUSH_GIMBAL_INFO,
                                                            GIMBAL_ADDRESS, BROADCAST_ADDRESS,
                                                            std::bind(&Gimbal::GimbalInfoCallback, this,
                                                                      std::placeholders::_1));
    handle_->CreateSubscriber<roborts_sdk::cmd_vision_info>(VISION_CMD_SET, CMD_VISION_MODE,
                                                            CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                            std::bind(&Gimbal::VisionInfoCallback, this,
                                                                      std::placeholders::_1));

    gimbal_angle_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_gimbal_angle>(GIMBAL_CMD_SET, CMD_SET_GIMBAL_ANGLE,
                                                                                MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
    gimbal_mode_pub_ = handle_->CreatePublisher<roborts_sdk::gimbal_mode_e>(GIMBAL_CMD_SET, CMD_SET_GIMBAL_MODE,
                                                                            MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
    fric_wheel_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_fric_wheel_speed>(GIMBAL_CMD_SET,
                                                                                  CMD_SET_FRIC_WHEEL_SPEED,
                                                                                  MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
    gimbal_shoot_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_shoot_info>(GIMBAL_CMD_SET, CMD_SET_SHOOT_INFO,
                                                                              MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);

    heartbeat_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_heartbeat>(UNIVERSAL_CMD_SET, CMD_HEARTBEAT,
                                                                          MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
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

void Gimbal::GimbalInfoCallback(const std::shared_ptr<roborts_sdk::cmd_gimbal_info> gimbal_info) {
    //记录云台相对于地盘的值
    //同样没有必要加锁
    pthread_spin_lock(&gimbal_to_chassis_lock);
    pthread_spin_unlock(&gimbal_to_chassis_lock);

}

void Gimbal::VisionInfoCallback(const std::shared_ptr<roborts_sdk::cmd_vision_info> vision_info) {
    current_mode = vision_info->vision_mode;
}


void Gimbal::RobotInfoCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_state> robot_info) {
    id = robot_info->robot_id;
    level = robot_info->robot_level;
}

void Gimbal::CtrlGimbalAngle(GimbalAngle msg) {

    if (handle_ == NULL)
        return;
    roborts_sdk::cmd_gimbal_angle gimbal_angle;
    gimbal_angle.ctrl.bit.pitch_mode = msg.pitch_mode;
    gimbal_angle.ctrl.bit.yaw_mode = msg.yaw_mode;
    gimbal_angle.pitch = msg.pitch_angle * 1800 / M_PI;
    gimbal_angle.yaw = msg.yaw_angle * 1800 / M_PI;
    //printf("%0.2f, %0.2f", gimbal_angle.yaw,gimbal_angle.pitch);
    gimbal_angle_pub_->Publish(gimbal_angle);

}

bool Gimbal::CtrlGimbalMode(int mode) {
    auto gimbal_mode = static_cast<roborts_sdk::gimbal_mode_e>(mode);
    gimbal_mode_pub_->Publish(gimbal_mode);
    return true;
}

bool Gimbal::CtrlFricWheel(bool open) {
    roborts_sdk::cmd_fric_wheel_speed fric_speed;
    if (open) {
        //打大幅用的射速
        fric_speed.left = 1280;
        fric_speed.right = 1280;
    } else {
        fric_speed.left = 1280;
        fric_speed.right = 1280;
    }
    fric_wheel_pub_->Publish(fric_speed);
    return true;
}

bool Gimbal::CtrlShoot(uint8_t mode, uint8_t number) {
    static std::chrono::steady_clock::time_point last_shoot = {};
    std::chrono::steady_clock::time_point current_shoot = std::chrono::steady_clock::now();
    //射击间隔,应该把0.1改成0.5(打大幅一秒两发)
    if (std::chrono::duration_cast<std::chrono::duration<double>>(current_shoot - last_shoot).count() > 0.1) {
        last_shoot = current_shoot;
    } else {
        return false;
    }

    roborts_sdk::cmd_shoot_info gimbal_shoot;
    uint16_t default_freq = 1500;
    switch (static_cast<roborts_sdk::shoot_cmd_e>(mode)) {
        case roborts_sdk::SHOOT_STOP:
            gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_STOP;
            gimbal_shoot.shoot_add_num = 0;
            gimbal_shoot.shoot_freq = 0;
            break;
        case roborts_sdk::SHOOT_ONCE:
            if (number != 0) {
                gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_ONCE;
                gimbal_shoot.shoot_add_num = number;
                gimbal_shoot.shoot_freq = default_freq;
            } else {
                gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_ONCE;
                gimbal_shoot.shoot_add_num = 1;
                gimbal_shoot.shoot_freq = default_freq;
            }
            break;
        case roborts_sdk::SHOOT_CONTINUOUS:
            gimbal_shoot.shoot_cmd = roborts_sdk::SHOOT_CONTINUOUS;
            gimbal_shoot.shoot_add_num = number;
            gimbal_shoot.shoot_freq = default_freq;
            break;
        default:
            return false;
    }
    gimbal_shoot_pub_->Publish(gimbal_shoot);
    return true;
}


//air friction is considered
float BulletModel(float x, float v, float angle) { //x:m,v:m/s,angle:rad
    float t, y;
    //t = (float)((exp(((double)INIT_K) / 1000 * x) - 1) / (((double)INIT_K) / 1000 * v * cos(angle)));
    t = x / v;
    y = (float) (v * sin(angle) * t - 9.78 * t * t / 2);
    return y;
}

//x:distance , y: height
float GetPitch(float x, float y, float v) {
    float y_temp, y_actual, dy;
    float a;
    y_temp = y;
    auto old_a = (float) atan2(y_temp, x);
    // by iteration
    for (int i = 0; i < 20; i++) {
        a = (float) atan2(y_temp, x);
        y_actual = BulletModel(x, v, a);
        dy = y - y_actual;
        y_temp = y_temp + dy;
        if (fabsf(dy) < 0.001) {
            break;
        }
        //printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n", i + 1, a * 180 / 3.1415926535, y_temp, dy);
    }
    //printf("GetPitch: %f, %f, %f\n", old_a, a, a - old_a);
    return a;

}


void Gimbal::AimAtArmor(const cv::Point3f &target_3d, bool compensation) {

    //用作校准相机,5000为归零
    OFFSET_X = (double)(OFFSET_INT_X - 5000);
    OFFSET_Y = (double)(OFFSET_INT_Y - 5000);
    OFFSET_Z = (double)(OFFSET_INT_Z - 5000);

    GimbalAngle gimbal_angle = {};

    if (target_3d.x == 0.0f && target_3d.y == 0.0f) {
        gimbal_angle.yaw_mode = true;
        gimbal_angle.pitch_mode = true;
        gimbal_angle.yaw_angle = 0.0f;
        gimbal_angle.pitch_angle = 0.0f;

        //如果把下面的注释打开,则当未检测到目标时会将云台停止
        //gimbal->CtrlGimbalAngle(gimbal_angle);
        //printf("pitch %f, yaw %f\n", gimbal_angle.pitch_angle,gimbal_angle.yaw_angle);
        return;
    }

    //将数值从毫米变为厘米

    float pitch = 0;
    if (compensation)
        //重力补偿
        pitch = -GetPitch((target_3d.z + OFFSET_Z / 100) / 100, -(target_3d.y + OFFSET_Y / 100) / 100, INIT_V) +
                (float) (OFFSET_PITCH * CV_PI / 180);
    else
        pitch = (float) (atan2(target_3d.y + OFFSET_Y / 100, target_3d.z + OFFSET_Z / 100) +
                         (float) (OFFSET_PITCH * CV_PI / 180));

    float yaw = -(float) (atan2(target_3d.x + OFFSET_X / 100, target_3d.z + OFFSET_Z / 100)) +
                (float) (OFFSET_YAW * CV_PI / 180);

    gimbal_angle.yaw_mode = true;
    gimbal_angle.pitch_mode = true;
    gimbal_angle.yaw_angle = (yaw) * 0.4f; //乘以0.7 or ratio来减慢云台yaw速度
    gimbal_angle.pitch_angle = pitch * 0.4f;
    //printf("pitch %f, yaw %f\n", gimbal_angle.pitch_angle,gimbal_angle.yaw_angle);
    CtrlGimbalAngle(gimbal_angle);
}

void print_gimbal_fps() {
    static time_t last_time = 0;
    static int show_count = 0;
    time_t current_time = time(NULL);
    if (current_time > last_time) {
        printf("gimbal fps: %d\n", show_count);
        last_time = current_time;
        show_count = 0;
    } else {
        show_count++;
    }
}
