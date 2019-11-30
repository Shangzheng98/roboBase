#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/h/rs_types.h>
#include <signal.h>

#include "roborts_sdk/sdk.h"
#include "gimbal.h"
#include "chassis.h"
#include "referee_system.h"
#include "armor_detection/armor_detection_node.h"
#include "iniparser/iniparser.h"
#include "auto_aiming.h"
#include "big_buff.h"
#include "msg.h"

//所有大写的全局变量均为参数,如果是变量则在配置文件里可以控制,这里是默认值(如果配置文件里没有)
/**
 * All capitalized variables can be controled in the configuration file(robot.ini)
 * hare are the default values
 */
int DEBUG_ENABLE = 1;
int ENEMY_COLOR = BLUE;


#define CONFIG_PATH "/home/jsz/Desktop/RobotBase/robot.ini"
//如果CONNECT_TO_SERIAL=0则不连接开发板,测试用

#define CONNECT_TO_SERIAL 0   //1 connect to development board
//0 not connect to development board, for test

Gimbal *gimbal = NULL;
Chassis *chassis = NULL;
rs2::pipeline *p_pipe = NULL;
roborts_sdk::cmd_game_robot_state robotID;


char SERIAL_PORT[20] = "/dev/ttyACM0";
int RS2_COLOR_FRAMERATE = 60;
int RS2_DEPTH_FRAMERATE = 60;
int RE2_FRAME_WIDTH = 640;
int RE2_FRAME_HEIGHT = 480;

int RES_HOLE_FILLING = false;

void print_frame_fps();

int load_param();

void control_panal();

//这个全局变量应该被开发板发送信息的回调函数改变
// the current_task can be changed by the callback function
int current_task = 1; //0 = auto aiming, 1 = big buff


int remain_time = 1;
int id = 1;


void *vision_main_function() {
    while (id == -1) {
        //blocks
    }
    //下面都是装甲识别的初始化工作
    // initialize the armor dectection
    ArmorDetectionNode armor_detection;
    armor_detection.Init();

    // set up the realsense camera
    rs2::config cfg;
    rs2::pipeline pipe;
    cfg.enable_stream(RS2_STREAM_COLOR, RE2_FRAME_WIDTH, RE2_FRAME_HEIGHT, RS2_FORMAT_BGR8, RS2_COLOR_FRAMERATE);
    cfg.enable_stream(RS2_STREAM_DEPTH, RE2_FRAME_WIDTH, RE2_FRAME_HEIGHT, RS2_FORMAT_Z16, RS2_DEPTH_FRAMERATE);
    rs2::pipeline_profile pipe_profile = pipe.start(cfg);

    rs2::stream_profile pipe_stream = pipe_profile.get_stream(RS2_STREAM_COLOR, -1);
    intrin_rgb = pipe_stream.as<rs2::video_stream_profile>().get_intrinsics();

    //初始化相机参数
    // init aiming bot
    auto_aiming_init(pipe_profile);

    rs2::align align_to_color(RS2_STREAM_COLOR);
    rs2::hole_filling_filter hole_filling_filter;
    hole_filling_filter.set_option(RS2_OPTION_HOLES_FILL, 2);


    int detection_started = 1;
    int task = 1;

    while (1) {

        rs2::frameset frames = pipe.wait_for_frames();

        if (!detection_started) {
            printf("Camera connected!\n");
            p_pipe = &pipe;


            armor_detection.StartThread();
            detection_started = 1;
        }
        if (task != current_task) {
            //如果current_task改变这里会执行一次,用作调整曝光,改变摩擦轮速度等初始操作
            task = current_task;

            switch (task) {
                case 0:
                    auto_aiming_init(pipe_profile);
                    break;
                case 1:
                    big_buff_init(pipe_profile);
                    break;
            }
        }
        switch (task) {
            case 0:
                auto_aiming(frames, align_to_color, hole_filling_filter, armor_detection);
                break;
            case 1:
                big_buff(frames);
                break;
        }
        //printf("the id: %d\n", robotID.robot_id);
        // printf("the id %d\n",id);
        print_frame_fps();
    }
}

int main(int argc, char *argv[]) {

    signal(SIGINT, [](int) {
        if (p_pipe) p_pipe->stop();
        _exit(0);
    });
    //读取参数,路径默认为/home/nvidia/robot.ini
    // load paramater from robot.ini
    load_param();

    //控制面板
    control_panal();
#if CONNECT_TO_SERIAL
    //启动SDK
    // start up the SDK
    auto handle = std::make_shared<roborts_sdk::Handle>(SERIAL_PORT);
    if (!handle->Init()) return 1;
    //初始化云台和地盘的控制
    //init  the gimbal contral and chassis contral
    gimbal = new Gimbal(handle);
    chassis = new Chassis(handle);
#else
    gimbal = new Gimbal(NULL);
#endif
    //视觉识别线程
    // CV thread
    std::thread vision_main_thread = std::thread(vision_main_function);
    //循环接收开发板发来的信息,主线程将为SDK服务
    // keep accecpting the message from development board, the main thread services SDk
    while (1) {
#if CONNECT_TO_SERIAL
        handle->Spin();
#endif
        usleep(1000);
    }

    return 0;
}


void print_frame_fps() {
    static time_t last_time = 0;
    static int show_count = 0;
    time_t current_time = time(NULL);
    if (current_time > last_time) {
        printf("frame fps: %d\n", show_count);
        last_time = current_time;
        show_count = 0;
    } else {
        show_count++;
    }
}

int load_param() {
    dictionary *ini = iniparser_load(CONFIG_PATH);
    if (ini == NULL) {
        printf("ERROR: cannot parse file: %s\n", CONFIG_PATH);
        return -1;
    }
    iniparser_dump(ini, stderr);
    int tmp_int = 0;
    double tmp_double = 0;
    const char *tmp_string = NULL;
    tmp_double = iniparser_getdouble(ini, "aiming:offset_x", -1);
    if (tmp_double != -1) OFFSET_X = tmp_double;
    tmp_double = iniparser_getdouble(ini, "aiming:offset_y", -1);
    if (tmp_double != -1) OFFSET_Y = tmp_double;
    tmp_double = iniparser_getdouble(ini, "aiming:offset_z", -1);
    if (tmp_double != -1) OFFSET_Z = tmp_double;
    tmp_double = iniparser_getdouble(ini, "aiming:offset_pitch", -1);
    if (tmp_double != -1) OFFSET_PITCH = tmp_double;
    tmp_double = iniparser_getdouble(ini, "aiming:offset_yaw", -1);
    if (tmp_double != -1) OFFSET_YAW = tmp_double;
    tmp_int = iniparser_getint(ini, "aiming:armor_width", -1);
    if (tmp_int != -1) ARMOR_WIDTH = tmp_int;
    tmp_int = iniparser_getint(ini, "aiming:armor_height", -1);
    if (tmp_int != -1) ARMOR_HEIGHT = tmp_int;
    tmp_int = iniparser_getboolean(ini, "aiming:debug_enable", -1);
    if (tmp_int != -1) DEBUG_ENABLE = tmp_int;
    tmp_string = iniparser_getstring(ini, "aiming:enemy_color", "not_found");
    if (!strcasecmp(tmp_string, "RED")) ENEMY_COLOR = RED;
    else ENEMY_COLOR = BLUE;

    tmp_string = iniparser_getstring(ini, "base:dev_path", "not_found");
    if (strcmp(tmp_string, "not_found")) strcpy(SERIAL_PORT, tmp_string);
    tmp_int = iniparser_getint(ini, "base:rs2_color_framerate", -1);
    if (tmp_int != -1) RS2_COLOR_FRAMERATE = tmp_int;
    tmp_int = iniparser_getint(ini, "base:rs2_depth_framerate", -1);
    if (tmp_int != -1) RS2_DEPTH_FRAMERATE = tmp_int;
    tmp_int = iniparser_getint(ini, "base:rs2_frame_width", -1);
    if (tmp_int != -1) RE2_FRAME_WIDTH = tmp_int;
    tmp_int = iniparser_getint(ini, "base:rs2_frame_height", -1);
    if (tmp_int != -1) RE2_FRAME_HEIGHT = tmp_int;
    tmp_int = iniparser_getboolean(ini, "base:rs2_hole_filling", -1);
    if (tmp_int != -1) RES_HOLE_FILLING = tmp_int;

    tmp_int = iniparser_getint(ini, "bigbuff:redLowH1", -1);
    if (tmp_int != -1) redLowH1 = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:redHighH1", -1);
    if (tmp_int != -1) redHighH1 = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:redLowH2", -1);
    if (tmp_int != -1) redLowH2 = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:redHighH2", -1);
    if (tmp_int != -1) redHighH2 = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:blueLowH", -1);
    if (tmp_int != -1) blueLowH = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:blueHighH", -1);
    if (tmp_int != -1) blueHighH = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:LowS", -1);
    if (tmp_int != -1) LowS = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:HighS", -1);
    if (tmp_int != -1) HighS = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:LowV", -1);
    if (tmp_int != -1) LowV = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:HighV", -1);
    if (tmp_int != -1) HighV = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:dilation_size", -1);
    if (tmp_int != -1) dilation_size = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:erosion_size", -1);
    if (tmp_int != -1) erosion_size = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:center_size_min", -1);
    if (tmp_int != -1) center_size_min = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:center_size_max", -1);
    if (tmp_int != -1) center_size_max = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:circle_radius_min", -1);
    if (tmp_int != -1) circle_radius_min = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:circle_radius_max", -1);
    if (tmp_int != -1) circle_radius_max = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:estimate_angle", -1);
    if (tmp_int != -1) estimate_angle = tmp_int;
    tmp_int = iniparser_getint(ini, "bigbuff:estimate_height", -1);
    if (tmp_int != -1) estimate_height = tmp_int;
    //tmp_int = iniparser_getint(ini, "bigbuff:direction", -1);
    //if (tmp_int != -1) direction = tmp_int;

    iniparser_freedict(ini);
    return 0;
}

void control_panal() {
    if (DEBUG_ENABLE == 0) {
        return;
    }
    namedWindow("Control", cv::WINDOW_AUTOSIZE);

    //if (ENEMY_COLOR == RED)

    cv::createTrackbar("redLowH1", "Control", &redLowH1, 180);        //Hue (0 - 179)
    cv::createTrackbar("redHighH1", "Control", &redHighH1, 180);
    cv::createTrackbar("redLowH2", "Control", &redLowH2, 180);         //Hue (0 - 179)
    cv::createTrackbar("redHighH2", "Control", &redHighH2, 180);


    cv::createTrackbar("LowS", "Control", &LowS, 255);         //Saturation (0 - 255)
    cv::createTrackbar("HighS", "Control", &HighS, 255);

    cv::createTrackbar("LowV", "Control", &LowV, 255);         //Value (0 - 255)
    cv::createTrackbar("HighV", "Control", &HighV, 255);

    cv::createTrackbar("dialation_size", "Control", &dilation_size, 50);
    cv::createTrackbar("erodition_size", "Control", &erosion_size, 50);

    cv::createTrackbar("center_size_min", "Control", &center_size_min, 100);
    cv::createTrackbar("center_size_max", "Control", &center_size_max, 100);

    cv::createTrackbar("circle_radius_min", "Control", &circle_radius_min, 100);
    cv::createTrackbar("circle_radius_max", "Control", &circle_radius_max, 100);

    cv::createTrackbar("estimate_angle", "Control", &estimate_angle, 360);
    cv::createTrackbar("estimate_height", "Control", &estimate_height, 100);

    namedWindow("Offset", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("OFFSET_INT_X", "Offset", &OFFSET_INT_X, 10000);
    cv::createTrackbar("OFFSET_INT_Y", "Offset", &OFFSET_INT_Y, 10000);
    cv::createTrackbar("OFFSET_INT_Z", "Offset", &OFFSET_INT_Z, 10000);
    cv::createTrackbar("OFFSET_INT_PITCH", "Offset", &OFFSET_INT_YAW, 100);
    cv::createTrackbar("OFFSET_INT_YAW", "Offset", &OFFSET_INT_PITCH, 100);
}
