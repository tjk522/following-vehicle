#ifndef LIDAR_MANAGER_H
#define LIDAR_MANAGER_H

// 初始化雷达串口
void LidarManager_Init();

// FreeRTOS 雷达专职扫描任务
void Task_LidarLoop(void *pvParameters);

#endif