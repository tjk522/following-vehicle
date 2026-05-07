/**
 * @file LidarManager.cpp
 * @brief YDLIDAR X2N 激光雷达驱动 — 目标检测 + 全扫描帧采集
 *
 * 雷达每秒钟旋转 5-8 圈（5-8Hz），每圈产生约 360 个扫描点。
 * 每个点包含：距离 (mm) + 角度 (°)。
 *
 * 这个任务做了两件事：
 *   【任务 A】目标跟踪（给 Leader-Follower 用）：
 *     在车头正前方 ±30° 扇形区域内找最近的目标，
 *     结果写入 global_state.target_distance / target_angle / is_target_found
 *
 *   【任务 B】全扫描帧采集（给 SLAM 用）：
 *     把一整圈的所有点存到缓冲区，圈结束时用双缓冲指针交换
 *     写入 global_scan（MicroROS 读取发布 /scan 话题）
 *
 *   双缓冲设计（关键！）：
 *     两块 buffer 交替使用：write_buf 正在写 → sweep 完成 →
 *     global_scan.ranges 指向刚写完的 buffer → write_buf 切到另一块
 *     临界区内只交换指针（微秒级），绝不 memcpy 数据
 */

#include "LidarManager.h"
#include <Arduino.h>
#include <GlobalData.h>
#include "TriLidar.h"

// 雷达串口引脚
#define LIDAR_RX_PIN 40
#define LIDAR_TX_PIN 41

TriLidar lidar;

// ============ 目标检测参数 ============
const float MIN_VALID_DIST = 100.0;   // 过滤 10cm 以内噪点（可能是扫到了车体）
const float MAX_VALID_DIST = 1500.0;  // 视距 1.5 米，太远的不追
const float ROI_ANGLE = 30.0;         // 关注正前方 ±30°（共 60° 扇形）

void LidarManager_Init() {
    lidar.begin(Serial2, 115200);
    // ESP32 硬件串口 2，115200 是 YDLIDAR X2N 的默认波特率
    Serial2.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    Serial.println("Lidar Driver Initialized!");
}

/**
 * @brief 雷达感知任务（Core 1，优先级 4）
 *
 * 实现原理：
 *   1. 逐点读取雷达数据（距离 + 角度）
 *   2. 检测跳变沿：last_angle - raw_angle > 180° → 说明雷达转完了一圈
 *   3. 每圈结束时：发布目标跟踪结果 + 交换扫描缓冲区指针
 *   4. 坐标系旋转：雷达的 0° 不一定在车头，需要加偏置角
 *   5. 容错机制：连续 3 圈没看到目标才认为"丢失"（避免单圈误判）
 */
void Task_LidarLoop(void *pvParameters) {
    float current_sweep_min_dist = 9999.0;
    float current_sweep_best_angle = 0.0;
    bool  found_in_this_sweep = false;
    float last_angle = 0.0;

    const float RADAR_YAW_OFFSET = 180.0;  // 雷达安装偏置角

    // 容错计数器：连续 N 圈没看到才算丢失
    int missed_sweeps_count = 0;
    const int MAX_MISSED_SWEEPS = 3;

    // ============ 双缓冲区（避免临界区内拷贝大数据）============
    static float buf_a[LIDAR_SCAN_POINTS], buf_b[LIDAR_SCAN_POINTS];
    float *write_buf = buf_a;          // 当前正在写入的 buffer
    uint16_t scan_buf_count = 0;       // 本圈已写入的点数
    bool use_buf_a = true;             // 当前在用哪个 buffer
    memset(write_buf, 0, sizeof(buf_a));

    while (1) {
        if (lidar.waitScanDot() == RESULT_OK) {
            float distance = lidar.getCurrentScanPoint().distance;  // mm
            float raw_angle = lidar.getCurrentScanPoint().angle;    // 度

            // ---- 跳变沿检测（一圈结束的结算时刻）----
            // 雷达角度从 360 跳变到 0 时，last_angle 约 359，raw_angle 约 0
            // 359 - 0 = 359 > 180 → 触发
            if (last_angle - raw_angle > 180.0) {

                // A. 写入目标跟踪数据
                taskENTER_CRITICAL(&state_spinlock);
                if (found_in_this_sweep) {
                    global_state.target_distance = current_sweep_min_dist;
                    global_state.target_angle    = current_sweep_best_angle;
                    global_state.is_target_found = true;
                    missed_sweeps_count = 0;  // 看到了，重置丢失计数
                } else {
                    missed_sweeps_count++;
                    // 连续多圈没看到，才承认目标真的丢了
                    if (missed_sweeps_count >= MAX_MISSED_SWEEPS) {
                        global_state.is_target_found = false;
                    }
                }
                taskEXIT_CRITICAL(&state_spinlock);

                // B. 双缓冲指针交换（临界区内只换指针，微秒级完成）
                taskENTER_CRITICAL(&scan_spinlock);
                global_scan.ranges = write_buf;
                global_scan.num_points = scan_buf_count;
                global_scan.scan_ready = true;
                taskEXIT_CRITICAL(&scan_spinlock);

                // 切换到另一个 buffer，清零准备新一圈
                use_buf_a = !use_buf_a;
                write_buf = use_buf_a ? buf_a : buf_b;

                current_sweep_min_dist = 9999.0;
                found_in_this_sweep = false;
                scan_buf_count = 0;
                memset(write_buf, 0, sizeof(buf_a));
            }
            last_angle = raw_angle;

            // ---- 坐标系旋转（雷达 0° → 车头 0°）----
            float car_angle = raw_angle + RADAR_YAW_OFFSET;
            if (car_angle >= 360.0) car_angle -= 360.0;
            else if (car_angle < 0.0) car_angle += 360.0;

            // ---- 存入扫描缓冲区（2° 分辨率，180 个点 / 圈）----
            uint16_t idx = (uint16_t)roundf(car_angle / 2.0f);
            if (idx < LIDAR_SCAN_POINTS) {
                write_buf[idx] = distance / 1000.0f;  // mm → m
                scan_buf_count++;
            }

            // ---- 扇形视野过滤（目标检测）----
            // 只关心正前方 ±30° 范围
            bool is_in_front = (car_angle <= ROI_ANGLE) ||
                              (car_angle >= (360.0 - ROI_ANGLE));
            bool is_valid_distance = (distance > MIN_VALID_DIST) &&
                                     (distance < MAX_VALID_DIST);

            if (is_in_front && is_valid_distance) {
                // 找 ROI 内最近的那个目标
                if (distance < current_sweep_min_dist) {
                    current_sweep_min_dist = distance;
                    current_sweep_best_angle = car_angle;
                    found_in_this_sweep = true;
                }
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);  // 1ms 间隔，不占满 CPU
    }
}
