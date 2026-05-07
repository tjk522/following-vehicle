#include "LeaderFollow.h"
#include <GlobalData.h>

// Leader-Follower 控制参数
const float TARGET_FOLLOW_DIST = 500.0f;   // 期望跟随距离 (mm)
const float MIN_SAFE_DIST      = 250.0f;   // 最小安全距离 (mm)，太近就后退
const float KP_DIST  = 0.0008f;            // 距离 PID 的 P
const float KP_ANGLE = 0.03f;              // 角度 PID 的 P
const float MAX_VX   = 0.2f;               // 最大线速度 (m/s)
const float MAX_W    = 1.0f;               // 最大角速度 (rad/s)

// 这个外部变量由 MicroROS_Node 的 follow_callback 控制
extern bool leader_follow_enabled;

void LeaderFollow_Init() {
    // nothing to init
}

void Task_LeaderFollow(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 50 / portTICK_PERIOD_MS; // 20Hz

    while (1) {
        if (leader_follow_enabled) {
            // 读取雷达目标
            float dist = 0, angle_deg = 0;
            bool found = false;
            taskENTER_CRITICAL(&state_spinlock);
            dist  = global_state.target_distance;  // mm
            angle_deg = global_state.target_angle;   // 度
            found = global_state.is_target_found;
            taskEXIT_CRITICAL(&state_spinlock);

            float vx = 0, w = 0;

            if (found) {
                float dist_err = dist - TARGET_FOLLOW_DIST; // mm

                if (dist < MIN_SAFE_DIST) {
                    // 太近！后退
                    vx = -0.1f;
                    w = 0;
                } else {
                    // 距离控制
                    vx = KP_DIST * dist_err;  // m/s
                    vx = constrain(vx, -MAX_VX, MAX_VX);

                    // 角度控制：目标偏左 → 正角速度(左转)；偏右 → 负角速度(右转)
                    // 角度约定：0=正前方，顺时针增加。左转为正角速度。
                    float angle_err = 0;
                    if (angle_deg <= 180.0f) {
                        // 目标在右侧 (顺时针方向)
                        angle_err = -angle_deg;
                    } else {
                        // 目标在左侧
                        angle_err = 360.0f - angle_deg;
                    }
                    w = KP_ANGLE * angle_err;
                    w = constrain(w, -MAX_W, MAX_W);
                }
            }
            // 没找到目标 → vx=0, w=0 (原地等待)

            // 写入全局黑板
            taskENTER_CRITICAL(&cmd_spinlock);
            global_cmd.vx = vx;
            global_cmd.vy = 0;
            global_cmd.w  = w;
            global_cmd.last_update_time_us = micros();
            taskEXIT_CRITICAL(&cmd_spinlock);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
