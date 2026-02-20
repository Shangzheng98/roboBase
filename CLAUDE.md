# roboBase 代码总结与优化分析

## 项目概述

**roboBase** 是用于 DJI RoboMaster 机器人竞赛的实时视觉瞄准系统。系统从大恒工业相机采集图像，运行两条并行检测流水线（装甲板 / 大符），通过 PnP 位姿估计计算 3D 坐标，并经串口向 STM32 主控发送云台控制指令。

---

## 技术栈

| 组件 | 技术 |
|------|------|
| 语言 | C++17 |
| 构建 | CMake 3.5+ |
| 计算机视觉 | OpenCV 4.1.1 |
| 多线程 | `std::thread` + Boost.Thread + POSIX pthread |
| 帧缓冲 | `boost::circular_buffer` |
| 日志 | glog（Google Logging） |
| JSON 解析 | RapidJSON |
| 相机 SDK | 大恒 GxIAPI |
| 串口通信 | POSIX termios（Linux） |
| 目标平台 | Jetson Nano（ARM aarch64）/ Ubuntu 桌面（x86_64） |

---

## 目录结构

```
roboBase/
├── CMakeLists.txt                  # 根构建配置
├── config.json                     # 相机标定参数、云台偏置
├── README.md
├── setUp_in_Nano.sh                # Jetson Nano 环境安装脚本（含 CUDA OpenCV）
├── setUp_NOT_in_Nano.sh            # Ubuntu 桌面环境安装脚本
├── evn_setup_test.sh               # 环境测试脚本
└── RobotBase/
    ├── main.cpp                    # 程序入口，启动 4 个线程
    ├── common.h                    # 公共枚举与数据结构
    ├── Robogrinder_SDK/            # 串口通信模块
    │   ├── message.h               # 通信协议结构体
    │   ├── serial_port.h
    │   └── serial_port.cpp
    ├── ThreadManagement/           # 线程调度模块
    │   ├── Thread_management.h
    │   └── Thread_management.cpp
    └── vision/
        ├── control.h               # 调试宏开关
        ├── cam/                    # 相机驱动
        │   ├── Daheng.h / .cpp     # 大恒相机初始化与采图
        │   ├── GxIAPI.h            # 大恒相机 API 头文件
        │   └── DxImageProc.h       # 大恒图像处理 API
        ├── autoAim/                # 自动瞄准模块
        │   ├── autoAim.h / .cpp    # 装甲板检测与位姿解算
        │   ├── armor.h / .cpp      # Armor / LED_bar 数据结构
        └── bigbuff/                # 大符检测模块
            ├── BigbuffDetection.h / .cpp
            └── pred_algrsm.h / .cpp  # 运动预测算法（头文件存在，实现未见）
```

---

## 系统架构

```
main()
  │
  ├─ std::thread ImageProduce()     大恒相机采图 → 写入 circular_buffer
  ├─ std::thread AutoAim()          读 buffer → 装甲板检测 → 串口发云台指令
  ├─ std::thread Bigbuff()          读 buffer → 大符检测 → 串口发云台指令
  └─ std::thread Communication()    键盘输入：'a' 切自瞄 / 'b' 切大符 / 'q' 退出
```

**数据流：**
1. `ImageProduce` 以死循环采帧，推入容量 30 的环形缓冲区（`boost::circular_buffer<cv::Mat>`）
2. `AutoAim` / `Bigbuff` 通过条件变量等待激活，激活后从缓冲区取帧处理
3. 检测到目标后调用 `solvePnP` 估计 3D 坐标，计算 pitch/yaw 角度
4. 通过 `/dev/ttyUSB0`（115200 波特率）将云台指令发送给 STM32

---

## 模块详解

### 1. `main.cpp`

启动点，分配 `ThreadManagement` 对象，创建 4 个 `std::thread` 并 `join`。无异常处理。

---

### 2. `ThreadManagement`

**构造函数：**
- 初始化环形缓冲区（容量 30）
- 初始化大恒相机
- 打开串口 `/dev/ttyUSB0`
- 根据 `otherParam.id` 硬编码判断队伍颜色（id==13 → 蓝方，否则 → 红方）

**线程方法：**

| 方法 | 功能 |
|------|------|
| `ImageProduce()` | 循环调用 `daheng->getImage(frame)` 并 push 到 buffer |
| `AutoAim()` | 等待 `aim_ready==true`，取帧调用 `armorDetector.armorTask()` |
| `Bigbuff()` | 等待 `buff_ready==true`，取帧调用 `bigebufDetector.feed_im()` |
| `Communication_thread()` | `getchar()` 读键盘，'a'→自瞄，'b'→大符 |
| `pause(char x)` | 将对应 `ready` 标志置 false |
| `resume(char x)` | 将对应 `ready` 标志置 true 并 `notify_one` |
| `stop()` | **空实现** |

---

### 3. `vision/autoAim` — 装甲板检测

**`ArmorDetector::armorTask()`** — 主入口：
1. 获取 ROI（首次全帧；之后以上次目标为中心扩张）
2. 调用 `DetectArmor()`
3. 若检测到目标：`solvePnP` → 计算 pitch/yaw → 串口发送
4. 若未检测到：发送 (0, 0)

**`DetectArmor()`** — 核心检测流水线：
1. 颜色分离（红：R-G；蓝：B-R），灰度亮度分离
2. 双阈值二值化（`gray_th_=24`，`color_th_=13`）
3. 轮廓检测，拟合 `fitEllipse` → `LED_bar`
4. 角度过滤（|angle| < 30°），长宽比过滤
5. O(n²) 配对所有 LED 条，调用 `armor::max_match()` 选最优配对
6. 从候选装甲板中选最近 ROI 中心的一个作为目标
7. 提取 4 个角点，调用 `solvePnP`（SOLVEPNP_ITERATIVE）

**`GetRoi()`** — 自适应 ROI：
- 目标存在时以上次包围盒为基础 ×2~3.5 倍扩张
- `lost_count >= 15` 时退化为全帧搜索

**相机内参（硬编码）：**
```
fx=1293.53, fy=1293.93, cx=355.91, cy=259.19
k1=-0.213, k2=0.228, p1=0.002, p2=0.0006, k3=-0.756
```

---

### 4. `vision/bigbuff` — 大符检测

**`BigbufDetector::feed_im()`** — 主入口：
1. `filte_image()`：颜色分离 + 双阈值 + 膨胀/腐蚀形态学处理
2. `locate_target()`：`findContours` → `contour_valid()` 验证 → `fitLine` + 矩计算目标中心
3. 若帧缓冲满则调用 `make_prediction()`

**`make_prediction()`：**
- 从 `frame_buffer[0]` 取装甲点
- `solvePnP` 解算 3D 坐标
- 计算 pitch/yaw 并发送串口指令

---

### 5. `Robogrinder_SDK` — 串口通信

**协议格式（发送）：**
```
rawData[0] = 0xaf (header)
rawData[1] = 0x01 (id=1, 云台指令)
rawData[2] = pitch & 0xFF
rawData[3] = pitch >> 8
rawData[4] = yaw & 0xFF
rawData[5] = yaw >> 8
```

**注意：** `serial_port` 按值传递，每次调用创建副本。`restart_serial_port()` 为空实现，串口故障后无法恢复。

---

### 6. `vision/control.h` — 调试开关

```cpp
#define ROI_ENABLE         1   // 启用 ROI 优化
#define SHOW_BINART        0   // 显示二值图（注意拼写，见 Bug 列表）
#define SHOW_LIGHT_CONTOURS 1  // 显示 LED 轮廓
#define FAST_DISTANCE      0   // 使用快速距离计算（曼哈顿距离）
#define SHOW_FINAL_ARMOR   1   // 显示最终装甲板
#define SHOW_ROI           1   // 显示 ROI 框
#define SHOW_DRAW_SPOT     1   // 显示目标中心点
#define SHOW_LAST_TARGET   1   // 显示上次目标框
```

---

## 优化空间

### Bug（功能性错误）

#### BUG-1：`make_prediction` 中 yaw 偏置使用了 `OFFSET_PITCH`
**文件：** `vision/bigbuff/BigbuffDetection.cpp:186`
```cpp
// 当前错误代码：
int yaw = int((-atan2(target_3d.x, target_3d.z) + (float)(OFFSET_PITCH * CV_PI / 1800)) * 0.6 * 10000);
//                                                          ^^^^^^^^^^^^^ 应为 OFFSET_YAW
```
yaw 角的偏置量误用了 `OFFSET_PITCH`，导致 yaw 偏置始终错误。

---

#### BUG-2：`locate_target` 中 `frame_info` 未推入 `frame_buffer`
**文件：** `vision/bigbuff/BigbuffDetection.cpp:146-151`
```cpp
frame_info target;
target.center = c_R;
target.angular_vector = a_vector;
target.f_time = clock();
target.armor_points = armor_points;
return true;  // target 从未被 push 到 frame_buffer！
```
`frame_buffer` 永远为空，`make_prediction` 永远使用 `frame_buffer[0]` 的无效数据，大符预测功能实际上不工作。

---

#### BUG-3：条件变量存在竞态条件（TOCTOU）
**文件：** `ThreadManagement/Thread_management.cpp:47-52`
```cpp
// 当前错误代码：
while(!aim_ready) {                          // 在锁外检查 aim_ready
    std::unique_lock<std::mutex> lck_a(aim_mtx);
    cond_aim.wait(lck_a);
}
```
`aim_ready` 的读取在持锁之前，存在竞态。标准写法应使用带谓词的等待：
```cpp
// 正确写法：
std::unique_lock<std::mutex> lck_a(aim_mtx);
cond_aim.wait(lck_a, [this]{ return aim_ready; });
```

---

#### BUG-4：`armorTask` 缺少返回值
**文件：** `vision/autoAim/autoAim.cpp:246`
```cpp
int ArmorDetector::armorTask(cv::Mat &color, OtherParam other_param, serial_port sp)
```
函数声明返回 `int`，但函数体所有路径均无 `return` 语句，属于 C++ 未定义行为。

---

#### BUG-5：`armor::max_match` 中 L/R 分类索引错误
**文件：** `vision/autoAim/armor.cpp:71-77`
```cpp
void armor::max_match(std::vector<LED_bar> &LEDs, size_t i, size_t j) {
    RotatedRect R, L;
    if (LEDs[0].rect.center.x > LEDs[1].rect.center.x) {  // 错误：应使用 LEDs[i] 和 LEDs[j]
        R = LEDs[0].rect;
        L = LEDs[1].rect;
    } ...
```
函数接收 `i`、`j` 参数但对 L/R 分类时硬编码使用下标 0 和 1，导致分类结果与实际传入的 LED 条对不匹配。

---

#### BUG-6：`serial_recive_data.size` 未初始化
**文件：** `Robogrinder_SDK/message.h:26-31`
```cpp
struct serial_recive_data {
    char rawData[10];
    int head = 0xaf;
    int id = 3;
    int size;          // 无默认值，使用时未定义
};
```
`serial_port::recive_data()` 中 `read(fd, receiveData.rawData, receiveData.size)` 使用未初始化的 `size`。

---

### 性能优化

#### PERF-1：`imshow`/`waitKey` 在生产代码中无条件执行
**文件：** `vision/autoAim/autoAim.cpp:241-242`，`vision/bigbuff/BigbuffDetection.cpp:174-175`
```cpp
imshow("debug_img", debug_img);  // 每帧都执行，阻塞约 1ms
waitKey(1);
```
GUI 渲染和键盘等待在每帧都执行，严重影响检测帧率。应使用 `#if DEBUG` 宏包裹。

---

#### PERF-2：`debug_img = img.clone()` 每帧无条件执行
**文件：** `vision/autoAim/autoAim.cpp:55`
```cpp
debug_img = img.clone();  // 无论是否需要调试，每帧复制整张图像
```
全分辨率图像克隆（640×480 × 3 通道）在每次 `DetectArmor` 调用时执行，即使所有 `SHOW_*` 宏均为 0。应在 `#if` 块内懒加载。

---

#### PERF-3：`serial_port` 按值传递
**文件：** `vision/autoAim/autoAim.h:60`，`vision/autoAim/autoAim.cpp:246`
```cpp
int armorTask(cv::Mat &img, OtherParam other_param, serial_port sp);  // sp 按值拷贝
```
`serial_port` 对象每次调用 `armorTask`（及 `BigbufDetector` 构造函数）都被拷贝，包含文件描述符的 int 成员，不应廉价拷贝。应改为引用传递 `serial_port &sp`。

---

#### PERF-4：`is_suitable_size()` 中重复调用 `sqrt`
**文件：** `vision/autoAim/armor.cpp:43-46`
```cpp
auto light_dis = std::sqrt(
    (led_bars[0].rect.center.x - led_bars[1].rect.center.x) * ... +
    (led_bars[0].rect.center.y - led_bars[1].rect.center.y) * ...
);
if (... && light_dis < 575.0f)
```
可以用距离平方 `< 575.0f * 575.0f` 代替，避免 `sqrt` 调用。

---

### 代码质量

#### QUAL-1：相机标定矩阵重复硬编码
`autoAim.h:99-103` 和 `BigbuffDetection.h:71-76` 中包含完全相同的 `cameraMatrix` 和 `distCoeffs` 数据，而项目根目录已有 `config.json` 存储了标定参数，但从未被读取。

**建议：** 在程序启动时用 RapidJSON 读取 `config.json`，通过构造函数参数传入标定矩阵。

---

#### QUAL-2：`config.json` 从未被使用
项目依赖 RapidJSON，`config.json` 包含 `cameraMatrix`、`distCoeffs`、`YAW_OFFSET`、`PITCH_OFFSET`、`DEBUG_MODE` 等字段，但代码中从未解析此文件。所有参数均为硬编码。

---

#### QUAL-3：`restart_serial_port()` 为空实现
**文件：** `Robogrinder_SDK/serial_port.cpp:70-72`

串口断开后无法自动重连，实战中串口抖动将导致通信永久中断。

---

#### QUAL-4：`stop()` 为空实现 + 线程无法优雅退出
**文件：** `ThreadManagement/Thread_management.cpp:151-153`

所有线程均为 `while(1)` 死循环，无退出标志位。按 'q' 仅退出 `Communication_thread` 的 `getchar` 循环，其余线程继续运行直到进程被杀死。

---

#### QUAL-5：`recive_data` 中存在死变量
**文件：** `Robogrinder_SDK/serial_port.cpp:76`
```cpp
uint8_t buffer[10];  // 声明后从未使用
```

---

#### QUAL-6：`SHOW_BINART` 拼写错误
**文件：** `vision/control.h:9`
```cpp
#define SHOW_BINART 0   // 拼写错误，应为 SHOW_BINARY
```
虽然 `autoAim.cpp:68` 中引用的是同名宏 `SHOW_BINART`，两处一致所以功能上不影响，但命名混乱。

---

#### QUAL-7：机器人颜色逻辑硬编码
**文件：** `ThreadManagement/Thread_management.cpp:21-26`
```cpp
if (otherParam.id == 13) // blue 3
    otherParam.color = 0;
else
    otherParam.color = 1;
```
颜色判断依赖硬编码 id，且注释与代码不一致（"blue 3" 但 id==13？）。应从 STM32 通过串口动态获取。

---

#### QUAL-8：`IDLE` 模式从未实现
**文件：** `common.h:14`
```cpp
typedef enum _mode { BIGBUFF, AUTOAIM, IDLE } mode;
```
`IDLE` 模式在枚举中定义，但线程调度逻辑中从未处理，两个检测线程都只有 active/wait 两态。

---

#### QUAL-9：`locate_target` 中 `hierarcy` 拼写错误
**文件：** `vision/bigbuff/BigbuffDetection.cpp:103`
```cpp
std::vector<cv::Vec4i> hierarcy;  // 应为 hierarchy
```

---

### 设计层面

#### DESIGN-1：AutoAim 和 Bigbuff 共用同一帧缓冲区
两个检测线程从同一个 `circular_buffer` 中 `pop_front()`，任意时刻只有一个处于活跃状态，但切换模式的 `sleep(1)` 窗口期内两者均可能读取或丢弃帧，造成帧浪费。

建议为每个检测线程维护独立缓冲区，或使用发布-订阅模式（观察者模式）分发帧。

---

#### DESIGN-2：`Communication_thread` 使用键盘输入
实战机器人的模式切换应由 STM32 通过串口下发控制指令，而不是操作员键盘输入。`recive_data()` 接口已存在但从未在 `Communication_thread` 中调用。

---

## 优化优先级总结

| 优先级 | 问题 | 影响 |
|--------|------|------|
| P0（功能错误）| BUG-1：yaw 偏置用 OFFSET_PITCH | 大符 yaw 方向完全错误 |
| P0（功能错误）| BUG-2：frame_buffer 从未填充 | 大符预测完全失效 |
| P0（功能错误）| BUG-3：条件变量竞态 | 线程可能永久死锁 |
| P1（性能）| PERF-1：imshow 无条件执行 | 检测帧率严重下降 |
| P1（性能）| PERF-2：img.clone 无条件执行 | 每帧多余内存拷贝 |
| P1（性能）| PERF-3：serial_port 值传递 | 每帧拷贝含 fd 的对象 |
| P2（稳定性）| QUAL-3：串口无法重连 | 串口抖动后永久失联 |
| P2（稳定性）| QUAL-4：线程无法优雅退出 | 资源无法正确释放 |
| P2（代码质量）| QUAL-2：config.json 未读取 | 标定参数无法热更新 |
| P3（代码质量）| BUG-4：armorTask 无返回值 | UB（未定义行为） |
| P3（代码质量）| BUG-5：max_match 索引错误 | LED 配对 L/R 分类错误 |
| P3（代码质量）| BUG-6：size 未初始化 | 串口读取长度不确定 |
