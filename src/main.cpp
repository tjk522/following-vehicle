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

    DisplayManager_Init();
    ChassisSystem_Init();
    LidarManager_Init();

    xTaskCreatePinnedToCore(
        Task_LidarLoop, "LidarTask", 4096, NULL, 4, NULL, 1
    );

    xTaskCreatePinnedToCore(
        Task_LeaderFollow, "FollowTask", 4096, NULL, 3, NULL, 1
    );

    xTaskCreatePinnedToCore(
        Task_ChassisLoop, "ChassisTask", 8192, NULL, 5, NULL, 1
    );

    xTaskCreatePinnedToCore(
        Task_MicroROSLoop, "MicroROSTask", 16384, NULL, 2, NULL, 0
    );

    Serial.println("Dual Core Architecture Deployed! Ready for Command.");
}

void loop() {
    DisplayManager_Update();

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
