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

double OFFSET_X = 0;
double OFFSET_Y = 0;
double OFFSET_Z = 0;
double OFFSET_PITCH = 0;
double OFFSET_YAW = 0;

//弹速,比真实弹速小2-3可能效果更好
int INIT_V = 12;

double gimbal_to_chassis_angle = 0;
pthread_spinlock_t gimbal_to_chassis_lock = {};

rs2_intrinsics intrin_rgb;


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

    handle_->CreateSubscriber<roborts_sdk::cmd_game_state>(REFEREE_GAME_CMD_SET, CMD_GAME_STATUS,
                                                           CHASSIS_ADDRESS, MANIFOLD2_ADDRESS,
                                                           std::bind(&Gimbal::StageInfoCallback, this,
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
    gimbal_to_chassis_angle = gimbal_info->yaw_gyro_angle / 10;
    pthread_spin_unlock(&gimbal_to_chassis_lock);

}

void Gimbal::VisionInfoCallback(const std::shared_ptr<roborts_sdk::cmd_vision_info> vision_info) {
    current_mode = vision_info->vision_mode;
    //printf("task:%d\n", current_task);
}

void Gimbal::StageInfoCallback(const std::shared_ptr<roborts_sdk::cmd_game_state> game_info) {
    remain_time = game_info->stage_remain_time;

}

void Gimbal::RobotInfoCallback(const std::shared_ptr<roborts_sdk::cmd_game_robot_state> robot_info) {
    id = robot_info->robot_id;
    //printf("call back id %d\n",id);
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
    roborts_sdk::gimbal_mode_e gimbal_mode = static_cast<roborts_sdk::gimbal_mode_e>(mode);
    gimbal_mode_pub_->Publish(gimbal_mode);
    return true;
}

bool Gimbal::CtrlFricWheel(bool open) {
    roborts_sdk::cmd_fric_wheel_speed fric_speed;
    if (open) {
        //打大幅用的射速
        fric_speed.left = 1340;
        fric_speed.right = 1340;
    } else {
        fric_speed.left = 1230;
        fric_speed.right = 1230;
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
    float old_a = (float) atan2(y_temp, x);
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


void Gimbal::AimAtArmor(int x, int y, cv::Mat depth_frame, bool compensation, bool prediction, bool shoot) {

    //用作校准相机,5000为归零点
//    if (DEBUG_ENABLE) {
//        if (OFFSET_INT_X != 5000)
//            OFFSET_X = ((double) OFFSET_INT_X - 5000);
//        if (OFFSET_INT_Y != 5000)
//            OFFSET_Y = ((double) OFFSET_INT_Y - 5000);
//        if (OFFSET_INT_Z != 5000)
//            OFFSET_Z = ((double) OFFSET_INT_Z - 5000);
//        if (OFFSET_INT_YAW != 5000)
//            OFFSET_YAW = ((double) OFFSET_INT_YAW - 5000);
//        if (OFFSET_INT_PITCH != 5000)
//            OFFSET_PITCH = ((double) OFFSET_INT_PITCH - 5000);
//    }

    GimbalAngle gimbal_angle = {};

    if (x == 0 && y == 0) {
        gimbal_angle.yaw_mode = true;
        gimbal_angle.pitch_mode = true;
        gimbal_angle.yaw_angle = 0;
        gimbal_angle.pitch_angle = 0;

        //如果把下面的注释打开,则当未检测到目标时会将云台停止
        //gimbal->CtrlGimbalAngle(gimbal_angle);
        return;
    }
    cv::Point3f target_3d = {};
    float rgb_points[3] = {};
    float rgb_pixels[2] = {(float) x, (float) y};

    //如果没有传来深度图(大幅模式)则将距离设为7米
    uint16_t depth = depth_frame.size().area() ? depth_frame.at<uint16_t>(y, x) : 7000;
    if (!depth)
        //仅在有深度数据时才操作云台
        return;

    //将pixel投射为三维坐标点
    rs2_deproject_pixel_to_point(rgb_points, &intrin_rgb, rgb_pixels, depth);
    float ratio = 0;

    //将数值从毫米变为厘米
    target_3d.x = rgb_points[0] / 10;
    target_3d.y = rgb_points[1] / 10;
    target_3d.z = rgb_points[2] / 10;

    float pitch = 0;
    if (compensation)
        //重力补偿
        pitch = -GetPitch((target_3d.z + OFFSET_Z / 100) / 100, -(target_3d.y + OFFSET_Y / 100) / 100, INIT_V) +
                (float) (OFFSET_PITCH * 3.1415926535 / 180);
    else
        pitch = (float) (atan2(target_3d.y + OFFSET_Y / 100, target_3d.z + OFFSET_Z / 100) +
                         (float) (OFFSET_PITCH * 3.1415926535 / 180));

    float yaw = -(float) (atan2(target_3d.x + OFFSET_X / 100, target_3d.z + OFFSET_Z / 100)) +
                (float) (OFFSET_YAW * 3.1415926535 / 180);
    gimbal_angle.yaw_mode = true;
    gimbal_angle.pitch_mode = true;
    gimbal_angle.yaw_angle = yaw * 0.4; //乘以0.7 or ratio来减慢云台yaw速度
    gimbal_angle.pitch_angle = pitch * 0.5;

    CtrlGimbalAngle(gimbal_angle);

    //仅在瞄准到了才开火,有必要的话可以把这个限制去掉,只要看见了就打
//	if (shoot && gimbal_angle.pitch_angle < 0.01 && gimbal_angle.yaw_angle < 0.01)
//		gimbal->CtrlShoot(roborts_sdk::SHOOT_ONCE, 1);

    //print_gimbal_fps();
}

int Gimbal::getCurrentMode() {
    return current_mode;
}

int Gimbal::getRemainTime() {
    return remain_time;
}

int Gimbal::getId() {
    return id;
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
