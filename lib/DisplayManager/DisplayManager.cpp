/**
 * @file DisplayManager.cpp
 * @brief OLED 屏幕管理器 — 从全局黑板取数据喂给屏幕驱动
 *
 * 为什么不直接在 UI 任务里访问 global_state？
 *   屏幕刷新在 Core 0 的 loop() 里执行，而 global_state 由 Core 1 写入。
 *   如果不用 spinlock 就读取，可能读到"一半新一半旧"的撕裂数据
 *   （比如 odom_x 和 odom_y 来自不同时刻，导致坐标瞬移）。
 *
 *   所以这个函数先抢锁、一次性抄下需要的所有数据、放锁，
 *   然后用本地副本喂给屏幕对象。
 */

#include "DisplayManager.h"

FishBotDisplay display;  // 全局唯一的屏幕对象

void DisplayManager_Init() {
    display.updateVersionCode("TJK-v1.0.0");
    display.init();
}

void DisplayManager_Update() {
    // ---- 准备本地缓冲区 ----
    float current_v = 0, current_w = 0, current_vol = 0;
    int   current_wifi_status = 3;
    char  current_ip[16]   = {0};
    char  current_ssid[32]  = {0};
    char  current_pswd[32]  = {0};
    float wheel_target[4]   = {0};
    float wheel_actual[4]   = {0};
    float wheel_pwm[4]      = {0};
    float imu_gx = 0, imu_gy = 0, imu_gz = 0;
    float imu_ax = 0, imu_ay = 0, imu_az = 0;
    float imu_qw = 0, imu_qx = 0, imu_qy = 0, imu_qz = 0;

    // ---- 抢锁 → 一次性抄录全部需要的数据 → 放锁（自旋锁内只做最小操作）----
    taskENTER_CRITICAL(&state_spinlock);
    current_v           = global_state.linear_x_speed;
    current_w           = global_state.angular_speed;
    current_vol         = global_state.battery_voltage;
    current_wifi_status = global_state.wifi_status;
    strcpy(current_ip,   (const char*)global_state.wifi_ip);
    strcpy(current_ssid, (const char*)global_state.wifi_ssid);
    strcpy(current_pswd, (const char*)global_state.wifi_pswd);
    memcpy(wheel_target, (const void*)global_state.wheel_target_speed, sizeof(wheel_target));
    memcpy(wheel_actual, (const void*)global_state.wheel_actual_speed, sizeof(wheel_actual));
    memcpy(wheel_pwm,    (const void*)global_state.wheel_pwm_duty,      sizeof(wheel_pwm));
    imu_gx = global_state.imu_gyro_x; imu_gy = global_state.imu_gyro_y; imu_gz = global_state.imu_gyro_z;
    imu_ax = global_state.imu_acc_x;  imu_ay = global_state.imu_acc_y;  imu_az = global_state.imu_acc_z;
    imu_qw = global_state.imu_q_w;    imu_qx = global_state.imu_q_x;    imu_qy = global_state.imu_q_y;    imu_qz = global_state.imu_q_z;
    taskEXIT_CRITICAL(&state_spinlock);

    // ---- 喂数据给屏幕对象 ----
    display.updateBotLinear(current_v);
    display.updateBotAngular(current_w);
    display.updateBatteryInfo(current_vol);
    display.updateTransMode("udp_client");

    display.updateWIFISSID(String(current_ssid));
    display.updateWIFIPSWD(String(current_pswd));
    display.updateWIFIIp(String(current_ip));
    display.updateWIFIServerIp("172.20.10.3");

    display.updateWIFIInfo("wifi_state", (fishbot_wifi_status_t)current_wifi_status);

    // 调试页数据
    display.updateWheelDebug(wheel_target, wheel_actual, wheel_pwm);

    // IMU 四元数 → 欧拉角（方便人眼看）
    // 公式：标准四元数→欧拉角转换（来自 3D 图形学）
    float euler_roll  = atan2f(2.0f * (imu_qw * imu_qx + imu_qy * imu_qz),
                               1.0f - 2.0f * (imu_qx * imu_qx + imu_qy * imu_qy)) * 57.2958f;
    float euler_pitch = asinf(2.0f * (imu_qw * imu_qy - imu_qz * imu_qx)) * 57.2958f;
    float euler_yaw   = atan2f(2.0f * (imu_qw * imu_qz + imu_qx * imu_qy),
                               1.0f - 2.0f * (imu_qy * imu_qy + imu_qz * imu_qz)) * 57.2958f;
    display.updateImuRaw(imu_gx, imu_gy, imu_gz,
                         imu_ax, imu_ay, imu_az,
                         euler_roll, euler_pitch, euler_yaw);

    // 触发屏幕渲染（内部有 1Hz 节流阀，不会每次都刷）
    display.updateDisplay();
}
