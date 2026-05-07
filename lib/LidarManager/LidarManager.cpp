#include "LidarManager.h"
#include <Arduino.h>
#include <GlobalData.h>
#include "TriLidar.h"

// --- 硬件引脚配置 ---
#define LIDAR_RX_PIN 40
#define LIDAR_TX_PIN 41

TriLidar lidar;

// --- 雷达视野算法参数 ---
const float MIN_VALID_DIST = 100.0;
const float MAX_VALID_DIST = 1500.0;
const float ROI_ANGLE = 30.0;

void LidarManager_Init() {
    lidar.begin(Serial2, 115200);
    Serial2.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
    Serial.println("Lidar Driver Initialized!");
}

void Task_LidarLoop(void *pvParameters) {
    float current_sweep_min_dist = 9999.0;
    float current_sweep_best_angle = 0.0;
    bool  found_in_this_sweep = false;
    float last_angle = 0.0;

    const float RADAR_YAW_OFFSET = 180.0;
    int missed_sweeps_count = 0;
    const int MAX_MISSED_SWEEPS = 3;

    // 扫描缓冲区 (本地构建，完成后一次性写入全局黑板)
    float local_ranges[LIDAR_SCAN_POINTS];
    float local_angles[LIDAR_SCAN_POINTS];
    uint16_t local_count = 0;
    memset(local_ranges, 0, sizeof(local_ranges));

    while (1) {
        if (lidar.waitScanDot() == RESULT_OK) {
            float distance = lidar.getCurrentScanPoint().distance;
            float raw_angle = lidar.getCurrentScanPoint().angle;

            // 1. 跳变沿检测 (一圈结束)
            if (last_angle - raw_angle > 180.0) {

                // --- 写入目标跟踪数据 ---
                taskENTER_CRITICAL(&state_spinlock);
                if (found_in_this_sweep) {
                    global_state.target_distance = current_sweep_min_dist;
                    global_state.target_angle    = current_sweep_best_angle;
                    global_state.is_target_found = true;
                    missed_sweeps_count = 0;
                } else {
                    missed_sweeps_count++;
                    if (missed_sweeps_count >= MAX_MISSED_SWEEPS) {
                        global_state.is_target_found = false;
                    }
                }
                taskEXIT_CRITICAL(&state_spinlock);

                // --- 写入完整扫描帧 ---
                taskENTER_CRITICAL(&scan_spinlock);
                memcpy((void*)global_scan.ranges, local_ranges, sizeof(local_ranges));
                memcpy((void*)global_scan.angles, local_angles, sizeof(local_angles));
                global_scan.num_points = local_count;
                global_scan.scan_stamp_us = micros();
                global_scan.scan_ready = true;
                taskEXIT_CRITICAL(&scan_spinlock);

                // 重置
                current_sweep_min_dist = 9999.0;
                found_in_this_sweep = false;
                local_count = 0;
                memset(local_ranges, 0, sizeof(local_ranges));
            }
            last_angle = raw_angle;

            // 2. 坐标系旋转
            float car_angle = raw_angle + RADAR_YAW_OFFSET;
            if (car_angle >= 360.0) car_angle -= 360.0;
            else if (car_angle < 0.0) car_angle += 360.0;

            // 3. 存入扫描缓冲区 (用于 LaserScan 发布)
            uint16_t idx = (uint16_t)roundf(car_angle);
            if (idx < LIDAR_SCAN_POINTS) {
                if (distance > MIN_VALID_DIST && distance < MAX_VALID_DIST) {
                    local_ranges[idx] = distance / 1000.0f; // mm -> m
                    local_angles[local_count] = car_angle;
                    local_count++;
                }
            }

            // 4. 扇形视野过滤 (目标跟踪)
            bool is_in_front = (car_angle <= ROI_ANGLE) || (car_angle >= (360.0 - ROI_ANGLE));
            bool is_valid_distance = (distance > MIN_VALID_DIST) && (distance < MAX_VALID_DIST);

            if (is_in_front && is_valid_distance) {
                if (distance < current_sweep_min_dist) {
                    current_sweep_min_dist = distance;
                    current_sweep_best_angle = car_angle;
                    found_in_this_sweep = true;
                }
            }
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
