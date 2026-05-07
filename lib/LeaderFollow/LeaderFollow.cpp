/**
 * @file LeaderFollow.cpp
 * @brief Leader-Follower 自动跟随算法
 *
 * 原理：
 *   利用雷达检测到的最近目标（距离 + 角度），生成 cmd_vel 来控制底盘：
 *     - 距离控制：用 P 控制器调节线速度 vx，保持 ~0.5m 跟随距离
 *     - 角度控制：用 P 控制器调节角速度 w，始终对准目标
 *     - 太近保护：小于 0.25m 时主动后退
 *
 * 开关方式：
 *   通过 ROS 2 话题 /leader_follow (std_msgs/Bool) 远程控制
 *   发 true 开启跟随，发 false 关闭（恢复手动控制 /cmd_vel）
 */

#include "LeaderFollow.h"
#include <GlobalData.h>

// ---- 可调参数 ----
const float TARGET_DIST = 500.0f;    // 期望跟随距离 (mm)
const float MIN_SAFE_DIST = 250.0f;   // 最小安全距离 (mm)，太近就后退
const float KP_DIST  = 0.0008f;       // 距离 P 增益（mm 误差 → m/s 速度）
const float KP_ANGLE = 0.03f;         // 角度 P 增益（度误差 → rad/s 角速度）
const float MAX_VX   = 0.2f;          // 最大线速度 (m/s)
const float MAX_W    = 1.0f;          // 最大角速度 (rad/s)

// leader_follow_enabled 定义在 MicroROS_Node.cpp 中，
// 由 /leader_follow 话题回调修改
extern bool leader_follow_enabled;

void LeaderFollow_Init() {}

/**
 * @brief 跟随控制任务（Core 1，优先级 3，20Hz）
 *
 * 不启用时什么也不做，只休眠。
 * 启用后每 50ms 从 global_state 读取雷达目标，计算 vx 和 w 写入 global_cmd。
 */
void Task_LeaderFollow(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;  // 50ms = 20Hz

    while (1) {
        if (leader_follow_enabled) {
            // 从全局黑板读取雷达目标数据
            float dist = 0, angle_deg = 0;
            bool found = false;
            taskENTER_CRITICAL(&state_spinlock);
            dist = global_state.target_distance;   // mm
            angle_deg = global_state.target_angle;  // 度
            found = global_state.is_target_found;
            taskEXIT_CRITICAL(&state_spinlock);

            float vx = 0, w = 0;

            if (found) {
                float dist_err = dist - TARGET_DIST;  // 正 = 太远, 负 = 太近

                if (dist < MIN_SAFE_DIST) {
                    // 太近了！后退
                    vx = -0.1f;
                    w = 0;
                } else {
                    // 距离 P 控制：误差大 → 速度快
                    vx = KP_DIST * dist_err;
                    vx = constrain(vx, -MAX_VX, MAX_VX);

                    // 角度 P 控制：目标偏右 → 负角速度（右转）；偏左 → 正角速度（左转）
                    // 角度约定：0° = 正前方，角度顺时针增加
                    float angle_err = 0;
                    if (angle_deg <= 180.0f) {
                        angle_err = -angle_deg;       // 右侧，需要负 w
                    } else {
                        angle_err = 360.0f - angle_deg; // 左侧，需要正 w
                    }
                    w = KP_ANGLE * angle_err;
                    w = constrain(w, -MAX_W, MAX_W);
                }
            }
            // 没找到目标 → vx=0, w=0，原地等待

            // 写入全局黑板（Leader-Follow 和 /cmd_vel 共用 global_cmd）
            taskENTER_CRITICAL(&cmd_spinlock);
            global_cmd.vx = vx;
            global_cmd.vy = 0;
            global_cmd.w  = w;
            global_cmd.last_update_time_us = micros();  // 更新时间戳，喂看门狗
            taskEXIT_CRITICAL(&cmd_spinlock);
        }

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
