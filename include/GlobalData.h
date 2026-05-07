#ifndef GLOBAL_DATA_H
#define GLOBAL_DATA_H

#include <Arduino.h>
//上位机和下位机可通过这个实现跨核交流
// =========================================================================
// 1. ROS 下发给底盘的控制指令 (Core 0 写，Core 1 读)
// =========================================================================
typedef struct {
    // 目标速度设定值
    float vx; // 整体前行线速度 (m/s)
    float vy; // 整体横移线速度 (m/s，麦轮专属)
    float w;  // 整体自转角速度 (rad/s)

    // 通信看门狗时间戳 (极其重要！)
    // 记录这包指令是何时收到的。如果下位机发现这个时间戳距离现在已经超过了 500ms，
    // 说明上位机/网络卡死了，下位机必须强制刹车，防止“疯车”。
    uint64_t last_update_time_us; 
} TargetCommand;

// =========================================================================
// 2. 底盘上报给 ROS 的完整状态 (Core 1 写，Core 0 读)
// =========================================================================
typedef struct {
    // ---------- A. 航位推算里程计 (Odometry) ----------
    float odom_x;              // 世界坐标系 X 轴绝对位置 (m)
    float odom_y;              // 世界坐标系 Y 轴绝对位置 (m)
    float odom_yaw;            // 世界坐标系 绝对偏航角 (rad，取值范围 -π 到 π)
    
    float linear_x_speed;      // 机器人车体坐标系 X 轴瞬时线速度 (m/s)
    float linear_y_speed;      // 机器人车体坐标系 Y 轴瞬时线速度 (m/s)
    float angular_speed;       // 机器人车体坐标系 瞬时角速度 (rad/s)

    // ---------- B. IMU 原始传感器数据 (Sensor Fusion 备用) ----------
    // 角速度 (由 MPU6050 陀螺仪直接输出，通常单位需转为 rad/s)
    float imu_gyro_x;          
    float imu_gyro_y;          
    float imu_gyro_z;          
    
    // 线加速度 (由 MPU6050 加速度计输出，通常单位需转为 m/s^2)
    float imu_acc_x;           
    float imu_acc_y;           
    float imu_acc_z;           
    
    // 绝对姿态四元数 (用于 ROS 3D 可视化和高级 Nav2 导航)
    float imu_q_w;             
    float imu_q_x;             
    float imu_q_y;             
    float imu_q_z;             

    // ---------- C. 硬件与系统状态 (System Monitor) ----------
    float battery_voltage;     // 电池瞬时电压 (V)，低压报警用
    uint8_t error_code;        // 底层错误码：0为正常，1为IMU掉线，2为编码器异常等

    // ---------- D. 极其硬核的底层控制数据 (Debug 用) ----------
    float wheel_target_speed[4];  // 四个轮子的期望速度 (mm/s)
    float wheel_actual_speed[4];  // 四个轮子的实际测速 (mm/s)
    float wheel_pwm_duty[4];      // 四个轮子当前的输出控制量 (占空比 -100~100)

    // ==========================================
    // [新增] E. WiFi 与网络状态 (Core 0 写入，Core 1 读取显示)
    // ==========================================
    int wifi_status;
    char wifi_ip[16];
    char wifi_ssid[32];
    char wifi_pswd[32];

    // ==========================================
    // [新增] F. 雷达感知数据 (Core 1 雷达任务写入，Core 1 底盘任务读取)
    // ==========================================
    float target_distance;     // 锁定目标的距离 (毫米 mm)
    float target_angle;        // 锁定目标的角度 (度 °)
    bool  is_target_found;     // 视野内是否发现了目标
} RobotState;

// =========================================================================
// 3. 跨核全局变量声明 (使用 extern)
// =========================================================================
// =========================================================================
// 2.5 激光雷达完整扫描帧
// =========================================================================
#define LIDAR_SCAN_POINTS 180
typedef struct {
    float *ranges;
    uint16_t num_points;
    bool scan_ready;
    uint64_t scan_stamp_us;
} LidarScan;

extern volatile TargetCommand global_cmd;
extern volatile RobotState global_state;
extern volatile LidarScan global_scan;

// =========================================================================
// 4. FreeRTOS 跨核互斥自旋锁 (Spinlocks)
// =========================================================================
extern portMUX_TYPE cmd_spinlock;
extern portMUX_TYPE state_spinlock;
extern portMUX_TYPE scan_spinlock;

#endif // GLOBAL_DATA_H