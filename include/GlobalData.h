/**
 * @file GlobalData.h
 * @brief 跨核通信数据结构 — ESP32 两颗核心之间的"全局黑板"
 *
 * 架构说明：
 *   ESP32-S3 有两个 CPU 核心，每个核心跑独立的 FreeRTOS 任务。
 *   但它们需要共享数据——ROS 下发的速度指令、底盘上报的里程计等。
 *
 *   方案：全局变量 + 自旋锁 (spinlock)
 *     - "volatile" 告诉编译器：每次读取必须从内存拿，不能用寄存器缓存，
 *       因为另一个核心随时可能修改这个值。
 *     - 自旋锁保证：任何核心在读写这些变量时先"抢锁"，
 *       读写完立刻"放锁"（几微秒内完成），防止读到"一半新一半旧"的撕裂数据。
 *
 *   重要原则：自旋锁内只做最小操作（赋值、指针交换），绝不调用阻塞函数。
 */

#ifndef GLOBAL_DATA_H
#define GLOBAL_DATA_H

#include <Arduino.h>

// =========================================================================
// 1. ROS 下发给底盘的控制指令（Core 0 MicroROS 写 → Core 1 底盘读）
// =========================================================================
typedef struct {
    float vx;  // 车体 X 轴线速度 (m/s)，前进为正
    float vy;  // 车体 Y 轴线速度 (m/s)，左移为正（麦克纳姆轮才有横移能力）
    float w;   // 车体 Z 轴角速度 (rad/s)，逆时针为正

    // 【通信看门狗】记录这包指令是何时收到的。
    // 如果底盘发现距离现在已经超过 500ms，说明上位机/网络卡死了，
    // 必须强制刹车（所有轮子速度为 0），防止"疯车"冲出去。
    uint64_t last_update_time_us;
} TargetCommand;

// =========================================================================
// 2. 底盘上报给 ROS 的完整状态（Core 1 底盘写 → Core 0 MicroROS 读）
// =========================================================================
typedef struct {
    // ---- A. 航位推算里程计 (Odometry) ----
    // 世界坐标系中的绝对位置和姿态
    float odom_x;         // X 轴绝对位置 (m)
    float odom_y;         // Y 轴绝对位置 (m)
    float odom_yaw;       // 偏航角 (rad, -π ~ π)

    // 机器人车体坐标系瞬时速度
    float linear_x_speed; // X 轴线速度 (m/s)
    float linear_y_speed; // Y 轴线速度 (m/s)
    float angular_speed;  // Z 轴角速度 (rad/s)

    // ---- B. IMU 原始传感器数据 ----
    // 角速度（陀螺仪输出）
    float imu_gyro_x, imu_gyro_y, imu_gyro_z;
    // 线加速度（加速度计输出，单位 m/s^2）
    float imu_acc_x,  imu_acc_y,  imu_acc_z;
    // 绝对姿态四元数（用于 ROS 3D 可视化和 robot_localization EKF 融合）
    // 四元数为什么比欧拉角好？——欧拉角有万向节死锁问题，四元数没有
    float imu_q_w, imu_q_x, imu_q_y, imu_q_z;

    // ---- C. 硬件与系统状态 ----
    float battery_voltage;  // 电池电压 (V)
    uint8_t error_code;     // 0=正常, 1=IMU掉线, 2=编码器异常...

    // ---- D. 底层控制数据（Debug 用，也能显示到 OLED）----
    float wheel_target_speed[4];  // 4 个轮子的目标转速 (mm/s)
    float wheel_actual_speed[4];  // 4 个轮子的实际测速 (mm/s)
    float wheel_pwm_duty[4];      // 4 个轮子的 PWM 占空比 (-100 ~ 100)

    // ---- E. WiFi 网络状态（Core 0 写 → Core 1 读来显示到 OLED）----
    int  wifi_status;
    char wifi_ip[16];
    char wifi_ssid[32];
    char wifi_pswd[32];

    // ---- F. 雷达感知数据（Core 1 雷达任务写 → FollowTask 读）----
    float target_distance;  // 锁定目标的距离 (mm)
    float target_angle;     // 锁定目标的角度 (度)
    bool  is_target_found;  // 视野内是否发现了目标
} RobotState;

// =========================================================================
// 2.5 激光雷达完整扫描帧（Core 1 雷达写 → Core 0 MicroROS 读来发布 /scan）
//
// 设计要点：
//   ranges 是指针而不是数组——避免在 spinlock 里 memcpy 大块数据。
//   雷达任务用双缓冲写入，只交换指针（几微秒），绝不拷贝数据。
// =========================================================================
#define LIDAR_SCAN_POINTS 180  // 2° 分辨率，360° 分成 180 个点
typedef struct {
    float *ranges;           // 指向就绪缓冲区（距离值，单位 m）
    uint16_t num_points;     // 本帧有效点数
    bool scan_ready;         // 新一帧数据已就绪，等待 MicroROS 取走
    uint64_t scan_stamp_us;  // 帧时间戳 (微秒)
} LidarScan;

// =========================================================================
// 3. 跨核全局变量声明（extern — 真正的定义在 main.cpp 中）
// =========================================================================
extern volatile TargetCommand global_cmd;
extern volatile RobotState  global_state;
extern volatile LidarScan    global_scan;

// =========================================================================
// 4. FreeRTOS 跨核互斥自旋锁（Spinlock）
//
// 自旋锁工作原理：
//   - taskENTER_CRITICAL(&lock)：如果锁被其他核心持有，就一直"自旋"等待
//   - taskEXIT_CRITICAL(&lock)：释放锁
//   - 自旋期间本核心的中断被关闭！所以锁内操作必须极短（微秒级）
//   - 适合保护几个变量赋值，绝不适合拷贝大数组或调用阻塞函数
// =========================================================================
extern portMUX_TYPE cmd_spinlock;
extern portMUX_TYPE state_spinlock;
extern portMUX_TYPE scan_spinlock;

#endif // GLOBAL_DATA_H
