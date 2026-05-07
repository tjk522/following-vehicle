#include "ChassisSystem.h"
#include <GlobalData.h>
// 引入硬件库...
#include <Esp32PcntEncoder.h>
#include <Esp32McpwmMotor.h>
#include <PID.h>
#include <Kinematics.h>
#include <IMU.h>

// 实例化硬件对象
Esp32McpwmMotor motor;
Esp32PcntEncoder encoders[4];
PidController pid_controller[4];
Kinematics kinematics;
MPU6050 mpu(Wire);
ImuDriver imu(mpu);

void ChassisSystem_Init() {
    // --- IMU 初始化 ---
    imu.begin(12, 13); 

    // --- 电机与编码器引脚绑定  ---
    motor.attachMotor(0, 5, 4);
    motor.attachMotor(1, 15, 16);
    motor.attachMotor(2, 3, 8);
    motor.attachMotor(3, 46, 9);
    
    encoders[0].init(0, 6, 7);
    encoders[1].init(1, 18, 17); 
    encoders[2].init(2, 20, 19); 
    encoders[3].init(3, 11, 10);

    // --- PID 和 运动学大脑初始化 ---
    float kp = 0.3, ki = 0.02, kd = 0.0; // 官方完美参数
    uint32_t speed_factor = 76160;       // 我们为你推导的完美速度因子
    
    kinematics.set_motion_model(MOTION_OMNIDIRECTIONAL);
    kinematics.set_kinematic_param(216.0f, 177.0f); // 麦轮轴距参数 a 和 b

    for (int i = 0; i < 4; i++) {
        pid_controller[i].update_target(0.0);
        pid_controller[i].update_pid(kp, ki, kd);
        pid_controller[i].out_limit(-100.0, 100.0); // MCPWM 库接受 -100 到 100 的占空比
        kinematics.set_motor_param(i, speed_factor);
    }
}

// 3. FreeRTOS 底盘专职任务 (Core 1 运行)
void Task_ChassisLoop(void *pvParameters) {
    // 设定极其严格的 10ms (100Hz) 控制周期
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 10 / portTICK_PERIOD_MS;
    
    static float target_speed[4];
    static float out_motor_duty[4];

    while (1) {
        // [1] 感知层：读取传感器
        imu.update();
        imu_t imu_data;
        if (imu.isEnable()) imu.getImuDriverData(imu_data);
        kinematics.update_motor_ticks(micros(), encoders[0].getTicks(), encoders[1].getTicks(), encoders[2].getTicks(), encoders[3].getTicks());

        // [2] 决策层：从全局黑板抢锁，读取上位机发来的期望速度
        float cmd_vx = 0, cmd_vy = 0, cmd_w = 0;
        
        taskENTER_CRITICAL(&cmd_spinlock);
        cmd_vx = global_cmd.vx;
        cmd_vy = global_cmd.vy;
        cmd_w  = global_cmd.w;
        uint64_t last_time = global_cmd.last_update_time_us;
        taskEXIT_CRITICAL(&cmd_spinlock);

        // 【底层安全看门狗】：如果超过 0.5 秒没有收到新指令，强制停车
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

        // [3] 解算层：让 Kinematics 把整体速度拆成 4 个轮子的目标 mm/s
        kinematics.kinematic_inverse(cmd_vx * 1000.0f, cmd_vy * 1000.0f, cmd_w, 
                                     target_speed[0], target_speed[1], target_speed[2], target_speed[3]);

        // [4] 执行层：PID 闭环与电机发波
        for (int i = 0; i < 4; i++) {
            pid_controller[i].update_target(target_speed[i]);
            out_motor_duty[i] = pid_controller[i].update(kinematics.motor_speed(i));
            motor.updateMotorSpeed(i, out_motor_duty[i]);
        }

        // [5] 汇报层：把推算出来的里程计写回全局黑板
        odom_t odom = kinematics.odom();
        
        taskENTER_CRITICAL(&state_spinlock);
        global_state.odom_x = odom.x;
        global_state.odom_y = odom.y;
        global_state.linear_x_speed = odom.linear_x_speed;
        global_state.linear_y_speed = odom.linear_y_speed;
        global_state.angular_speed = odom.angular_speed;
        if (imu.isEnable()) {
            global_state.odom_yaw = imu_data.orientation_euler.z;
            global_state.imu_gyro_x = imu_data.angular_velocity.x;
            global_state.imu_gyro_y = imu_data.angular_velocity.y;
            global_state.imu_gyro_z = imu_data.angular_velocity.z;
            global_state.imu_acc_x  = imu_data.linear_acceleration.x;
            global_state.imu_acc_y  = imu_data.linear_acceleration.y;
            global_state.imu_acc_z  = imu_data.linear_acceleration.z;
            global_state.imu_q_w = imu_data.orientation.w;
            global_state.imu_q_x = imu_data.orientation.x;
            global_state.imu_q_y = imu_data.orientation.y;
            global_state.imu_q_z = imu_data.orientation.z;
        } else {
            global_state.odom_yaw = odom.yaw;
        }
        // === 新增：抄写四个轮子的底层运行数据 ===
        for(int i = 0; i < 4; i++) {
            global_state.wheel_target_speed[i] = target_speed[i];
            global_state.wheel_actual_speed[i] = kinematics.motor_speed(i);
            global_state.wheel_pwm_duty[i]     = out_motor_duty[i];
        }
        taskEXIT_CRITICAL(&state_spinlock);

        // 严格延时，让出 CPU 避免看门狗复位，保证 10ms 周期
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}