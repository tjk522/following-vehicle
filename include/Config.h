#ifndef CONFIG_H
#define CONFIG_H

// ==================== 电机引脚配置 ====================
// 0:左前(FL)  1:右前(FR)  2:左后(BL)  3:右后(BR)
#define MOTOR_FL_A 5
#define MOTOR_FL_B 4

#define MOTOR_FR_A 15
#define MOTOR_FR_B 16

#define MOTOR_BL_A 3
#define MOTOR_BL_B 8

#define MOTOR_BR_A 46
#define MOTOR_BR_B 9

// ==================== 编码器引脚配置 ====================
#define ENC_FL_A 6
#define ENC_FL_B 7

#define ENC_FR_A 18
#define ENC_FR_B 17

#define ENC_BL_A 20
#define ENC_BL_B 19

#define ENC_BR_A 11
#define ENC_BR_B 10

// ==================== 预留模块引脚 ====================
#define IMU_SDA_PIN 12
#define IMU_SCL_PIN 13

#define VOICE_TX_PIN -1
#define VOICE_RX_PIN -1

// ==================== 麦克纳姆轮物理参数 ====================
const float CHASSIS_LX = 108.0f; // 左右轮中心距的一半 (单位：m)
const float CHASSIS_LY = 88.5f; // 前后轮中心距的一半 (单位：m)

// --- 新增：电机与轮子参数 (请根据你的实物规格修改) ---
const float WHEEL_DIAMETER = 48.0f; // 麦克纳姆轮外直径 (mm) 常见为 65mm 或 80mm
const int REDUCTION_RATIO = 45;     // 电机减速比 (如 1:30 的电机填 30)
const int PULSE_RATIO = 11;         // 霍尔编码器电机轴转一圈的脉冲数

#endif