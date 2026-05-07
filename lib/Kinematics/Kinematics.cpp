#include "Kinematics.h"

void Kinematics::Euler2Quaternion(float roll, float pitch, float yaw, quaternion_t &q)
{
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

// 【数学工具】约束角度域：让偏航角永远保持在 [-π, π] 之间。
// 比如转了 370 度，它会自动变成 10 度。防止角度无限变大导致三角函数计算溢出。
void Kinematics::TransAngleInPI(float angle, float &out_angle)
{
    if (angle > PI)
    {
        out_angle -= 2 * PI;
    }
    else if (angle < -PI)
    {
        out_angle += 2 * PI;
    }
}

// void Kinematics::set_motor_param(uint8_t id, uint16_t reducation_ratio, uint16_t pulse_ration, float wheel_diameter)
// {
//     motor_param_[id].id = id;
//     motor_param_[id].reducation_ratio = reducation_ratio;
//     motor_param_[id].pulse_ration = pulse_ration;
//     motor_param_[id].wheel_diameter = wheel_diameter;
//     motor_param_[id].per_pulse_distance = (wheel_diameter * PI) / (reducation_ratio * pulse_ration);
//     motor_param_[id].speed_factor = (1000 * 1000) * (wheel_diameter * PI) / (reducation_ratio * pulse_ration);
//     // 给初值
//     motor_param_[id].last_encoder_tick = 0;
//     Serial.printf("init motor param %d: %f=%f*PI/(%d*%d) speed_factor=%d\n", id, motor_param_[id].per_pulse_distance, wheel_diameter, reducation_ratio, pulse_ration, motor_param_[id].speed_factor);
// }

void Kinematics::set_motor_param(uint8_t id, uint32_t speed_factor)
{
    motor_param_[id].id = id;
    motor_param_[id].speed_factor = speed_factor;
    motor_param_[id].last_encoder_tick = 0;
    Serial.printf("init motor param %d: speed_factor=%d\n", id, motor_param_[id].speed_factor);
}

void Kinematics::set_kinematic_param(float wheel_distance)
{
    wheel_distance_ = wheel_distance;
}
void Kinematics::set_kinematic_calib(float calib_mx, float calib_dx, float calib_myaw, float calib_dyaw)
{
    calib_mx_ = calib_mx;
    calib_dx_ = calib_dx;
    calib_myaw_ = calib_myaw;
    calib_dyaw_ = calib_dyaw;
}

// 【时间流转中心】：这里将枯燥的脉冲变化，变成了真实的物理速度
void Kinematics::update_motor_ticks(uint64_t current_time, int32_t motor_tick1, int32_t motor_tick2, int32_t motor_tick3, int32_t motor_tick4)
{
    // 1. 极其严谨的计算逝去的时间 dt (微秒级别)
    uint32_t dt = current_time - motor_param_[0].last_update_time;
    
    // 声明 static 数组，避免每次调用函数都在栈内存里重新分配，提高 C++ 执行效率
    static int32_t dticks[4];
    static int32_t motor_ticks[4];
    static int8_t index;

    motor_ticks[0] = motor_tick1;
    motor_ticks[1] = motor_tick2;
    motor_ticks[2] = motor_tick3;
    motor_ticks[3] = motor_tick4;

    for (int index = 0; index < 4; index++)
    {
        // 2. 算出这短短 dt 时间内，多出来的脉冲数
        dticks[index] = motor_ticks[index] - motor_param_[index].last_encoder_tick;
        /// 3. 计算真实物理速度 (mm/s)。这就是前面为什么要提前算好 speed_factor 的原因！
        // 速度 = (Δ脉冲 / Δ时间) * 单脉冲距离。
        // 这里把乘除法通过结合律做了极致优化，避免了在主循环里做大量浮点运算。
        motor_param_[index].motor_speed = dticks[index] * (motor_param_[index].speed_factor / dt);
        // 4. 更新历史记忆，为下一个周期做准备
        motor_param_[index].last_encoder_tick = motor_ticks[index];
        motor_param_[index].last_update_time = current_time;
    }

    // 更新机器人里程计（推算小车在世界里的位置）
    this->update_bot_odom_(dt);
}


// 【核心里程计积分】：航位推算 (Dead Reckoning)
// 我刚才以某个速度跑了 dt 秒，我现在在哪？
void Kinematics::update_bot_odom_(uint32_t dt)
{
    static float linear_x_speed, linear_y_speed, angular_speed;
    // 把微秒 dt 转换成国际标准单位：秒 (s)
    float dt_s = (float)(dt / 1000) / 1000;
    // 1. 调用正运动学，把四个轮子的测速，变成小车整体的线速度和角速度
    this->kinematic_forward(motor_param_[0].motor_speed,
                            motor_param_[1].motor_speed,
                            motor_param_[2].motor_speed,
                            motor_param_[3].motor_speed,
                            linear_x_speed,
                            linear_y_speed,
                            angular_speed);

    // 2. 转换成国际标准单位 (把 mm/s 变成 m/s) 存入 Odom
    odom_.angular_speed = angular_speed;
    odom_.linear_x_speed = linear_x_speed / 1000; // /1000（mm/s 转 m/s）
    odom_.linear_y_speed = linear_y_speed / 1000; // /1000（mm/s 转 m/s）

    // 3. 打滑补偿
    if (motion_motion_ == MOTION_OMNIDIRECTIONAL)
    {
        odom_.linear_x_speed *= calib_mx_;
        odom_.angular_speed *= calib_myaw_;
    }
    else
    {
        odom_.linear_x_speed *= calib_dx_;
        odom_.angular_speed *= calib_dyaw_;
    }
    /*更新x和y轴上移动的距离*/
    odom_.x += (odom_.linear_x_speed * cos(odom_.yaw) - odom_.linear_y_speed * sin(odom_.yaw)) * dt_s;
    odom_.y += (odom_.linear_x_speed * sin(odom_.yaw) + odom_.linear_y_speed * cos(odom_.yaw)) * dt_s;
    // 角度积分：新角度 = 老角度 + 角速度 * 时间
    odom_.yaw += odom_.angular_speed * dt_s;
    // 约束角度，防止超过 360 度
    Kinematics::TransAngleInPI(odom_.yaw, odom_.yaw);
    // Serial.printf("odom(%f,%f)\n", odom_.x, odom_.y);
}

// 【逆运动学 Inverse Kinematics】：大脑下发指令
// 输入：期望的 Vx, Vy, W。 输出：四个轮子该转多快。
void Kinematics::kinematic_inverse(float linear_x_speed, float linear_y_speed, float angular_speed,
                                   float &out_wheel_speed1, float &out_wheel_speed2, float &out_wheel_speed3, float &out_wheel_speed4)
{
    if (motion_motion_ == MOTION_OMNIDIRECTIONAL)
    {
        out_wheel_speed1 = linear_x_speed - linear_y_speed - angular_speed * (wheel_distance_a_and_b_);
        out_wheel_speed2 = linear_x_speed + linear_y_speed + angular_speed * (wheel_distance_a_and_b_);
        out_wheel_speed3 = linear_x_speed + linear_y_speed - angular_speed * (wheel_distance_a_and_b_);
        out_wheel_speed4 = linear_x_speed - linear_y_speed + angular_speed * (wheel_distance_a_and_b_);
        // Serial.printf("out_wheel_speed[%f,%f,%f,%f]\n", out_wheel_speed1, out_wheel_speed2, out_wheel_speed3, out_wheel_speed4);
    }
    else if (motion_motion_ == MOTION_DIFFERENTIAL_DRIVE)
    {
        out_wheel_speed1 = linear_x_speed - (angular_speed * wheel_distance_ / 2);
        out_wheel_speed3 = linear_x_speed - (angular_speed * wheel_distance_ / 2);
        out_wheel_speed2 = linear_x_speed + (angular_speed * wheel_distance_ / 2);
        out_wheel_speed4 = linear_x_speed + (angular_speed * wheel_distance_ / 2);
    }
}

// 【正运动学 Forward Kinematics】：大脑感知现实
// 输入：四个轮子的真实转速。 输出：推算出车体现在的 Vx, Vy, W。
void Kinematics::kinematic_forward(float wheel1_speed, float wheel2_speed, float wheel3_speed, float wheel4_speed,
                                   float &linear_x_speed, float &linear_y_speed, float &angular_speed)
{
    if (motion_motion_ == MOTION_OMNIDIRECTIONAL)
    {
        linear_x_speed = (wheel1_speed + wheel2_speed + wheel3_speed + wheel4_speed) / 4.0f;
        linear_y_speed = (-wheel1_speed + wheel2_speed + wheel3_speed - wheel4_speed) / 4.0f;
        angular_speed = float(-wheel1_speed + wheel2_speed - wheel3_speed + wheel4_speed) / (4.0f * (wheel_distance_a_and_b_));
    }
    else if (motion_motion_ == MOTION_DIFFERENTIAL_DRIVE)
    {
        // 计算机器人的 x 轴线速度，公式为四个轮子转速之和的平均值。
        linear_x_speed = (wheel1_speed + wheel2_speed + wheel3_speed + wheel4_speed) / 4.0f;
        linear_y_speed = 0.0f; // For differential drive, there is no lateral movement
        angular_speed = (wheel2_speed + wheel4_speed - wheel1_speed - wheel3_speed) / (2.0f * wheel_distance_);
    }
    // Serial.printf("angular_speed:%f wheel_speed[%f,%f,%f,%f]\n",angular_speed, wheel1_speed, wheel2_speed, wheel3_speed, wheel4_speed);
}

odom_t &Kinematics::odom()
{
    Kinematics::Euler2Quaternion(0, 0, odom_.yaw, odom_.quaternion);
    return odom_;
}
void Kinematics::set_motion_model(motion_model_t model)
{
    motion_motion_ = model;
}
float Kinematics::motor_speed(uint8_t id)
{
    return motor_param_[id].motor_speed;
}
void Kinematics::set_kinematic_param(float wheel_distance_a, float wheel_distance_b)
{
    wheel_distance_a_ = wheel_distance_a;
    wheel_distance_b_ = wheel_distance_b;
    wheel_distance_a_and_b_ = (wheel_distance_a_ + wheel_distance_b_) / 2;
}