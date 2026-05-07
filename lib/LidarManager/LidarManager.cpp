#include "LidarManager.h"
#include <Arduino.h>
#include <GlobalData.h>
#include "TriLidar.h"

#define LIDAR_RX_PIN 40
#define LIDAR_TX_PIN 41

TriLidar lidar;

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

    // 双缓冲 ranges（angles 不需要，LaserScan 只用到 ranges）
    static float buf_a[LIDAR_SCAN_POINTS], buf_b[LIDAR_SCAN_POINTS];
    float *write_buf = buf_a;
    uint16_t scan_buf_count = 0;
    bool use_buf_a = true;
    memset(write_buf, 0, sizeof(buf_a));

    while (1) {
        if (lidar.waitScanDot() == RESULT_OK) {
            float distance = lidar.getCurrentScanPoint().distance;
            float raw_angle = lidar.getCurrentScanPoint().angle;

            if (last_angle - raw_angle > 180.0) {
                // 目标跟踪
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

                taskENTER_CRITICAL(&scan_spinlock);
                global_scan.ranges = write_buf;
                global_scan.num_points = scan_buf_count;
                global_scan.scan_ready = true;
                taskEXIT_CRITICAL(&scan_spinlock);

                use_buf_a = !use_buf_a;
                write_buf = use_buf_a ? buf_a : buf_b;

                current_sweep_min_dist = 9999.0;
                found_in_this_sweep = false;
                scan_buf_count = 0;
                memset(write_buf, 0, sizeof(buf_a));
            }
            last_angle = raw_angle;

            float car_angle = raw_angle + RADAR_YAW_OFFSET;
            if (car_angle >= 360.0) car_angle -= 360.0;
            else if (car_angle < 0.0) car_angle += 360.0;

            // 存入扫描缓冲区
            uint16_t idx = (uint16_t)roundf(car_angle / 2.0f);
            if (idx < LIDAR_SCAN_POINTS) {
                write_buf[idx] = distance / 1000.0f;
                scan_buf_count++;
            }

            // 目标跟踪过滤
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
