/**
 * @file ChassisSystem.cpp
 * @brief 底盘控制系统 — 100Hz PID 闭环控制（跑在 Core 1）
 *
 * 控制流程（5 层架构，每 10ms 执行一次）：
 *   [1] 感知层：读 IMU + 编码器 → 算出四个轮子的真实转速
 *   [2] 决策层：从全局黑板读取 ROS 下发的目标速度（带看门狗）
 *   [3] 解算层：逆运动学 — 车体 vx/vy/w → 四个轮子的目标转速
 *   [4] 执行层：PID 闭环 — 目标转速 vs 实际转速 → PWM 占空比 → 电机
 *   [5] 汇报层：里程计/IMU 数据写回全局黑板 → MicroROS 发布到上位机
 *
 * 关键概念：
 *   - 逆运动学 (Inverse Kinematics)：知道车体怎么走 → 算出每个轮子该转多快
 *   - PID 控制：目标值 vs 实际值，用 P(比例)/I(积分)/D(微分) 三项算出控制量
 *   - 看门狗 (Watchdog)：超过 500ms 没收指令就刹车，防止"疯车"
 *   - 里程计 (Odometry)：根据轮速推算小车在世界坐标系中的位置（航位推算）
 */

#include "ChassisSystem.h"
#include <GlobalData.h>
#include <Esp32PcntEncoder.h>
#include <Esp32McpwmMotor.h>
#include <PID.h>
#include <Kinematics.h>
#include <IMU.h>

// ================ 硬件对象实例化 ================
// MCPWM = 用硬件 PWM 模块驱动电机，比 analogWrite 精准得多
Esp32McpwmMotor motor;
// PCNT = ESP32 的脉冲计数器硬件模块，不占 CPU
Esp32PcntEncoder encoders[4];
// 每个轮子一个 PID 控制器
PidController pid_controller[4];
// 运动学大脑：负责逆运动学、正运动学、里程计推算
Kinematics kinematics;
// MPU6050 六轴传感器（三轴陀螺仪 + 三轴加速度计）
MPU6050 mpu(Wire);
ImuDriver imu(mpu);

/**
 * @brief 底盘硬件初始化
 *
 * 执行顺序很重要：
 *   1. IMU 先初始化（I2C 总线启动）
 *   2. 电机引脚绑定
 *   3. 编码器初始化（每个编码器独占一个 PCNT 硬件单元）
 *   4. PID 参数设置（kp=0.3, ki=0.02, kd=0 是调好的参数）
 *   5. 运动学模型设为"全向轮"（麦克纳姆轮）
 *
 * speed_factor 计算推导：
 *   轮径 48mm，减速比 45:1，编码器 11 线 4 倍频
 *   轮子每转一圈的脉冲数 = 45 × 11 × 4 = 1980
 *   轮子周长 = 48 × π ≈ 150.8 mm
 *   每个脉冲对应距离 = 150.8 / 1980 ≈ 0.076 mm
 *   speed_factor = 0.076 × 10^6 ≈ 76160 (us → s 转换)
 */
void ChassisSystem_Init() {
    // IMU 用 I2C 总线，SDA=12, SCL=13
    imu.begin(12, 13);

    // 电机引脚：attachMotor(编号, A相, B相)
    // 编号 0=左前FL, 1=右前FR, 2=左后BL, 3=右后BR
    motor.attachMotor(0, 5, 4);
    motor.attachMotor(1, 15, 16);
    motor.attachMotor(2, 3, 8);
    motor.attachMotor(3, 46, 9);

    // 编码器：init(PCNT单元号, A相引脚, B相引脚)
    encoders[0].init(0, 6, 7);
    encoders[1].init(1, 18, 17);
    encoders[2].init(2, 20, 19);
    encoders[3].init(3, 11, 10);

    float kp = 0.3, ki = 0.02, kd = 0.0;
    uint32_t speed_factor = 76160;

    // 设置运动模型为麦克纳姆轮（全向轮）
    kinematics.set_motion_model(MOTION_OMNIDIRECTIONAL);
    // 传入麦轮轴距参数：a = 左右轮距和/2, b = 前后轮距和/2
    kinematics.set_kinematic_param(216.0f, 177.0f);

    for (int i = 0; i < 4; i++) {
        pid_controller[i].update_target(0.0);
        pid_controller[i].update_pid(kp, ki, kd);
        // MCPWM 库的占空比范围是 -100 到 100
        pid_controller[i].out_limit(-100.0, 100.0);
        kinematics.set_motor_param(i, speed_factor);
    }
}

/**
 * @brief 底盘 PID 控制任务 — 100Hz 硬实时循环（Core 1 最高优先级）
 *
 * vTaskDelayUntil 保证精确的固定周期，不受循环体执行时间抖动影响。
 * 和普通 vTaskDelay 的区别：
 *   - vTaskDelay(100ms)：从调用时起等 100ms，如果循环体用了 5ms，实际周期是 105ms
 *   - vTaskDelayUntil：保证绝对周期，每 10ms 绝对唤醒一次
 */
void Task_ChassisLoop(void *pvParameters) {
    // 记录第一次唤醒的时间，之后每次严格加 10ms
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;  // 10ms = 100Hz

    static float target_speed[4];   // 四个轮子的目标转速 (mm/s)
    static float out_motor_duty[4]; // 四个轮子的 PWM 输出 (-100 ~ 100)

    while (1) {
        // =============== [1] 感知层：读取传感器 ===============
        imu.update();  // 触发 MPU6050 读取最新数据
        imu_t imu_data;
        if (imu.isEnable()) {
            // 一次性把陀螺仪、加速度计、四元数全读出来
            imu.getImuDriverData(imu_data);
        }
        // 喂四个编码器的脉冲累计值给运动学大脑 → 算出每个轮子的真实转速
        kinematics.update_motor_ticks(
            micros(),
            encoders[0].getTicks(), encoders[1].getTicks(),
            encoders[2].getTicks(), encoders[3].getTicks()
        );

        // =============== [2] 决策层：读上位机指令 ===============
        float cmd_vx = 0, cmd_vy = 0, cmd_w = 0;
        taskENTER_CRITICAL(&cmd_spinlock);
        cmd_vx = global_cmd.vx;
        cmd_vy = global_cmd.vy;
        cmd_w  = global_cmd.w;
        uint64_t last_time = global_cmd.last_update_time_us;
        taskEXIT_CRITICAL(&cmd_spinlock);

        // 【通信看门狗】超过 500ms 没收指令 → 强制停车
        // 同时 reset PID 历史状态，防止积分项累积导致松刹车后猛冲
        static bool was_braking = false;
        if (micros() - last_time > 500000) {
            cmd_vx = 0; cmd_vy = 0; cmd_w = 0;
            if (!was_braking) {
                for (int i = 0; i < 4; i++) pid_controller[i].reset();
                was_braking = true;
            }
        } else {
            was_braking = false;
        }

        // =============== [3] 解算层：逆运动学 ===============
        // 输入：车体 vx(m/s), vy(m/s), w(rad/s)
        // 输出：四个轮子各自该转多快 (mm/s)
        // 注意单位转换：m/s × 1000 = mm/s
        kinematics.kinematic_inverse(
            cmd_vx * 1000.0f, cmd_vy * 1000.0f, cmd_w,
            target_speed[0], target_speed[1],
            target_speed[2], target_speed[3]
        );

        // =============== [4] 执行层：PID 闭环控制 ===============
        // PID 原理：
        //   P(比例)：误差越大，输出越大 — 快速接近目标
        //   I(积分)：累积历史误差 — 消除稳态偏差
        //   D(微分)：预测未来趋势 — 抑制震荡
        // 公式：output = Kp×e(k) + Ki×Σe + Kd×(e(k)-e(k-1))
        for (int i = 0; i < 4; i++) {
            pid_controller[i].update_target(target_speed[i]);
            // 当前转速 (反馈) → PID 计算 → PWM 输出
            out_motor_duty[i] = pid_controller[i].update(
                kinematics.motor_speed(i)
            );
            motor.updateMotorSpeed(i, out_motor_duty[i]);
        }

        // =============== [5] 汇报层：写回全局黑板 ===============
        // Core 1 → global_state → Core 0 读走 → ROS /odom, /imu
        odom_t odom = kinematics.odom();

        taskENTER_CRITICAL(&state_spinlock);
        global_state.odom_x = odom.x;
        global_state.odom_y = odom.y;
        global_state.linear_x_speed = odom.linear_x_speed;
        global_state.linear_y_speed = odom.linear_y_speed;
        global_state.angular_speed = odom.angular_speed;

        if (imu.isEnable()) {
            // IMU 正常 → 用真实 Yaw（磁力计融合过的，不会漂移）
            global_state.odom_yaw = imu_data.orientation_euler.z;

            // 完整的 IMU 数据（给 ROS /imu 话题和 OLED 显示）
            global_state.imu_gyro_x = imu_data.angular_velocity.x;
            global_state.imu_gyro_y = imu_data.angular_velocity.y;
            global_state.imu_gyro_z = imu_data.angular_velocity.z;
            global_state.imu_acc_x  = imu_data.linear_acceleration.x;
            global_state.imu_acc_y  = imu_data.linear_acceleration.y;
            global_state.imu_acc_z  = imu_data.linear_acceleration.z;

            // 四元数（给 ROS 3D 可视化和 robot_localization 融合用）
            global_state.imu_q_w = imu_data.orientation.w;
            global_state.imu_q_x = imu_data.orientation.x;
            global_state.imu_q_y = imu_data.orientation.y;
            global_state.imu_q_z = imu_data.orientation.z;
        } else {
            // IMU 挂了 → 退而用编码器推算的 Yaw（会漂移但不至于飞）
            global_state.odom_yaw = odom.yaw;
        }

        // 四轮调试数据（给 OLED 调试页显示）
        for (int i = 0; i < 4; i++) {
            global_state.wheel_target_speed[i] = target_speed[i];
            global_state.wheel_actual_speed[i] = kinematics.motor_speed(i);
            global_state.wheel_pwm_duty[i]     = out_motor_duty[i];
        }
        taskEXIT_CRITICAL(&state_spinlock);

        // 精确延时到下一个 10ms 时刻
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}
