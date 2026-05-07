#include "Debug.h"
#include <Chassis.h>

void Debug_EncoderTest() {
    // 1. 给极小的开环 PWM 让电机慢速旋转
    Chassis_SetSingleMotor(0, 50); 
    Chassis_SetSingleMotor(1, 50);
    Chassis_SetSingleMotor(2, 50);
    Chassis_SetSingleMotor(3, 50);

    // 2. 使用 static 变量实现非阻塞延时，每 100ms 刷新一次
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime >= 100) {
        lastPrintTime = millis();
        
        long cFL = 0, cFR = 0, cBL = 0, cBR = 0;
        Chassis_GetEncoderCounts(cFL, cFR, cBL, cBR);
        
        // 将具体的数值变化打印在命令行窗口中
        Serial.printf("Encoders -> FL:%ld FR:%ld BL:%ld BR:%ld\n", cFL, cFR, cBL, cBR);
    }
}