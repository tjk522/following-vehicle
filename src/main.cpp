/**
 * @file main.cpp
 * @brief FishBot 主程序 — 双核 FreeRTOS 架构
 *
 * ESP32-S3 有两个 CPU 核心，这里用 FreeRTOS 把任务分别绑定到不同核心：
 *   Core 0: WiFi + Micro-ROS 通信 + 屏幕刷新
 *   Core 1: 底盘 PID 控制 + 雷达感知 + Leader-Follower 跟随
 *
 * 跨核通信通过"全局黑板 + 自旋锁"实现：
 *   - Core 0 写 cmd_vel（目标速度）→ Core 1 读
 *   - Core 1 写 odom/IMU/雷达数据 → Core 0 读来发布到 ROS
 *
 * volatile 关键字告诉编译器："每次读取必须从内存拿，别用寄存器缓存"，
 * 因为另一个核心可能随时修改这个变量。
 * 自旋锁 (spinlock) 保证读/写时不会出现"数据撕裂"（读到一半新一半旧）。
 */

#include <Arduino.h>
#include <GlobalData.h>
#include "ChassisSystem.h"
#include "MicroROS_Node.h"
#include "DisplayManager.h"
#include "LidarManager.h"
#include "LeaderFollow.h"

// ================ 跨核全局黑板（volatile = 每次从内存读取）================
volatile TargetCommand global_cmd = {0.0f, 0.0f, 0.0f, 0};
volatile RobotState  global_state = {0};
volatile LidarScan    global_scan  = {0};

// ================ FreeRTOS 自旋锁（保护上面的全局变量）================
// portMUX_INITIALIZER_UNLOCKED = 初始化为"未上锁"状态
portMUX_TYPE cmd_spinlock   = portMUX_INITIALIZER_UNLOCKED; // 保护 global_cmd
portMUX_TYPE state_spinlock = portMUX_INITIALIZER_UNLOCKED; // 保护 global_state
portMUX_TYPE scan_spinlock  = portMUX_INITIALIZER_UNLOCKED; // 保护 global_scan

void setup() {
    Serial.begin(115200);

    // 让 ESP32 有时间稳定下来，避免启动时序问题
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // 初始化各个硬件模块
    DisplayManager_Init();  // OLED 屏幕
    ChassisSystem_Init();   // 电机、编码器、PID、IMU
    LidarManager_Init();    // YDLIDAR 激光雷达

    // ==================== Core 1 任务（实时控制核心）====================
    // xTaskCreatePinnedToCore 参数含义：
    //   任务函数, 任务名, 栈大小(字节), 参数, 优先级(越大越高), 句柄, 核心号

    // LidarTask: 激光雷达数据采集 + 目标检测（优先级 4）
    xTaskCreatePinnedToCore(
        Task_LidarLoop, "LidarTask", 4096, NULL, 4, NULL, 1
    );

    // FollowTask: 根据雷达目标自动跟随（优先级 3，低于底盘控制）
    xTaskCreatePinnedToCore(
        Task_LeaderFollow, "FollowTask", 4096, NULL, 3, NULL, 1
    );

    // ChassisTask: 100Hz PID 底盘控制 — 硬实时，最高优先级（5）
    xTaskCreatePinnedToCore(
        Task_ChassisLoop, "ChassisTask", 8192, NULL, 5, NULL, 1
    );

    // ==================== Core 0 任务（通信核心）====================
    // MicroROSTask: WiFi 连接 + Micro-ROS 节点（优先级 2）
    xTaskCreatePinnedToCore(
        Task_MicroROSLoop, "MicroROSTask", 16384, NULL, 2, NULL, 0
    );

    Serial.println("Dual Core Architecture Deployed! Ready for Command.");
}

/**
 * @brief Core 0 主循环（优先级最低，有空就执行）
 *
 * 负责：
 *   1. 刷新 OLED 屏幕（1Hz 渲染，10Hz 按键检测）
 *   2. 打印雷达目标信息到串口监视器（调试用）
 *
 * vTaskDelay 让出 CPU，不会浪费算力空转。
 */
void loop() {
    // 屏幕刷新 + 按键检测（每 100ms 一次）
    DisplayManager_Update();

    // --- 雷达调试：从全局黑板抄数据打印到串口 ---
    float debug_dist = 0, debug_angle = 0;
    bool  debug_found = false;

    // 抢锁 → 抄数据 → 放锁（自旋锁内只做最小操作）
    taskENTER_CRITICAL(&state_spinlock);
    debug_dist  = global_state.target_distance;  // mm
    debug_angle = global_state.target_angle;      // 度
    debug_found = global_state.is_target_found;
    taskEXIT_CRITICAL(&state_spinlock);

    if (debug_found) {
        Serial.printf("[雷达锁定] 距离: %.1f mm, 角度: %.1f 度\n", debug_dist, debug_angle);
    } else {
        Serial.println("[雷达扫描] 前方 1.5 米内安全，无目标");
    }

    // 让出 CPU 100 毫秒（FreeRTOS 要求用 vTaskDelay，不能用 delay()）
    vTaskDelay(100 / portTICK_PERIOD_MS);
}
