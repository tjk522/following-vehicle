#include <Arduino.h>
#include <GlobalData.h>
#include "ChassisSystem.h"
#include "MicroROS_Node.h"
#include "DisplayManager.h"
#include "LidarManager.h"
#include "LeaderFollow.h"

volatile TargetCommand global_cmd = {0.0f, 0.0f, 0.0f, 0};
volatile RobotState global_state = {0};
volatile LidarScan global_scan = {0};
portMUX_TYPE cmd_spinlock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE state_spinlock = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE scan_spinlock = portMUX_INITIALIZER_UNLOCKED;

void setup() {
    Serial.begin(115200);
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    //初始化屏幕
    DisplayManager_Init();

    //  初始化底盘硬件
    ChassisSystem_Init();

    // [新增] 1. 初始化雷达串口
    LidarManager_Init();

    // [新增] 1. 启动 Core 1 雷达感知任务 (优先级 4)
    xTaskCreatePinnedToCore(
        Task_LidarLoop, "LidarTask", 4096, NULL, 4, NULL, 1
    );

    // [新增] 2. 启动 Leader-Follower 跟随任务 (优先级 3)
    xTaskCreatePinnedToCore(
        Task_LeaderFollow, "FollowTask", 4096, NULL, 3, NULL, 1
    );

    //  将底盘控制分配给 Core 1 (硬实时极高优先级)
    xTaskCreatePinnedToCore(
        Task_ChassisLoop, "ChassisTask", 8192, NULL, 5, NULL, 1 
    );
    
    //  将 Micro-ROS 通信分配给 Core 0 (处理 WiFi 与网络)
    xTaskCreatePinnedToCore(
        Task_MicroROSLoop, "MicroROSTask", 16384, NULL, 2, NULL, 0 
    );

    Serial.println("Dual Core Architecture Deployed! Ready for Command.");
}

void loop() {
    // UI 线程：刷新屏幕（内含按键检测）
    DisplayManager_Update();

    // 雷达调试打印
    float debug_dist = 0;
    float debug_angle = 0;
    bool  debug_found = false;

    taskENTER_CRITICAL(&state_spinlock);
    debug_dist  = global_state.target_distance;
    debug_angle = global_state.target_angle;
    debug_found = global_state.is_target_found;
    taskEXIT_CRITICAL(&state_spinlock);

    if (debug_found) {
        Serial.printf("[雷达锁定] 距离: %.1f mm, 角度: %.1f 度\n", debug_dist, debug_angle);
    } else {
        Serial.println("[雷达扫描] 前方 1.5 米内安全，无目标");
    }

    vTaskDelay(100 / portTICK_PERIOD_MS);
}
