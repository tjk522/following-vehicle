#ifndef CHASSIS_SYSTEM_H
#define CHASSIS_SYSTEM_H

#include <Arduino.h>

// 初始化所有的底盘硬件（电机、编码器、PID、IMU）
void ChassisSystem_Init();

// 专为 FreeRTOS 设计的独立任务函数（无限循环）
void Task_ChassisLoop(void *pvParameters);

#endif // CHASSIS_SYSTEM_H