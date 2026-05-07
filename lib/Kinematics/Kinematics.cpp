/**
 * @file Kinematics.cpp
 * @brief 麦克纳姆轮运动学 — 把"车体怎么走"翻译成"四个轮子转多快"
 *
 * 两类核心运算：
 *
 *   【逆运动学】kinematic_inverse()
 *     输入：车体期望速度 (vx, vy, w)
 *     输出：四个轮子的转速
 *     用途：控制器下发指令时用
 *
 *   【正运动学】kinematic_forward()
 *     输入：四个轮子的实际转速
 *     输出：车体实际速度 (vx, vy, w)
 *     用途：推算里程计时用
 *
 * 麦轮公式推导（轮子编号：0=左前FL, 1=右前FR, 2=左后BL, 3=右后BR）：
 *
 *   逆解：
 *     vFL = vx - vy - w×(a+b)   (左前轮)
 *     vFR = vx + vy + w×(a+b)   (右前轮)
 *     vBL = vx + vy - w×(a+b)   (左后轮)
 *     vBR = vx - vy + w×(a+b)   (右后轮)
 *
 *   正解：
 *     vx = (vFL + vFR + vBL + vBR) / 4
 *     vy = (-vFL + vFR + vBL - vBR) / 4
 *     w  = (-vFL + vFR - vBL + vBR) / (4×(a+b))
 *
 * 里程计积分（航位推算 Dead Reckoning）：
 *   已知 dt 秒前我在 (x, y, yaw)，以速度 (vx, vy, w) 移动了 dt 秒
 *   → x += (vx×cos(yaw) - vy×sin(yaw)) × dt   ← 旋转矩阵投影到世界系
 *   → y += (vx×sin(yaw) + vy×cos(yaw)) × dt
 *   → yaw += w × dt
 */

#include "Kinematics.h"

// ================ 纯数学工具函数 ================

/**
 * @brief 欧拉角 → 四元数转换
 *
 * 为什么要四元数？
 *   - 欧拉角 (Roll, Pitch, Yaw) 有万向节死锁问题（±90° 时系统退化）
 *   - 四元数没有死锁，且插值更平滑
 *   - ROS 的标准姿态表示就是四元数
 *
 * 公式来源：3D 图形学标准推导，把三次绕轴旋转合成一个四元数
 */
void Kinematics::Euler2Quaternion(float roll, float pitch, float yaw,
                                   quaternion_t &q) {
    double cr = cos(roll * 0.5),  sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    double cy = cos(yaw * 0.5),   sy = sin(yaw * 0.5);
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

// 角度归一化：把任意角度约束到 [-π, π]
void Kinematics::TransAngleInPI(float angle, float &out_angle) {
    if (angle > PI)       out_angle -= 2 * PI;
    else if (angle < -PI) out_angle += 2 * PI;
}

// ================ 参数设置 ================

void Kinematics::set_motor_param(uint8_t id, uint32_t speed_factor) {
    motor_param_[id].id = id;
    motor_param_[id].speed_factor = speed_factor;
    motor_param_[id].last_encoder_tick = 0;
}
void Kinematics::set_kinematic_param(float wd)          { wheel_distance_ = wd; }
void Kinematics::set_kinematic_param(float a, float b) {
    wheel_distance_a_ = a;
    wheel_distance_b_ = b;
    wheel_distance_a_and_b_ = (a + b) / 2;  // 逆/正运动学公式里用的是 (a+b)
}
void Kinematics::set_kinematic_calib(float mx, float dx, float myaw, float dyaw) {
    calib_mx_ = mx; calib_dx_ = dx;
    calib_myaw_ = myaw; calib_dyaw_ = dyaw;
}
void Kinematics::set_motion_model(motion_model_t model) { motion_motion_ = model; }
float Kinematics::motor_speed(uint8_t id) { return motor_param_[id].motor_speed; }

// ================ 逆运动学（大脑 → 轮子）================

void Kinematics::kinematic_inverse(float vx, float vy, float w,
                                    float &v1, float &v2,
                                    float &v3, float &v4) {
    if (motion_motion_ == MOTION_OMNIDIRECTIONAL) {
        // 麦克纳姆轮：4 个轮子各自有不同的转速组合
        v1 = vx - vy - w * wheel_distance_a_and_b_;  // FL
        v2 = vx + vy + w * wheel_distance_a_and_b_;  // FR
        v3 = vx + vy - w * wheel_distance_a_and_b_;  // BL
        v4 = vx - vy + w * wheel_distance_a_and_b_;  // BR
    } else {
        // 差速驱动（两轮/履带）：左右两边的轮子速度不同来实现转向
        v1 = vx - w * wheel_distance_ / 2;
        v3 = vx - w * wheel_distance_ / 2;
        v2 = vx + w * wheel_distance_ / 2;
        v4 = vx + w * wheel_distance_ / 2;
    }
}

// ================ 正运动学（轮子 → 车体）================

void Kinematics::kinematic_forward(float v1, float v2, float v3, float v4,
                                    float &vx, float &vy, float &w) {
    if (motion_motion_ == MOTION_OMNIDIRECTIONAL) {
        // 四个轮子的平均转速 = 车体前进速度
        vx = (v1 + v2 + v3 + v4) / 4.0f;
        // 对角线轮子转速差 = 横移速度
        vy = (-v1 + v2 + v3 - v4) / 4.0f;
        // 左右轮转速差 = 旋转角速度
        w  = (-v1 + v2 - v3 + v4) / (4.0f * wheel_distance_a_and_b_);
    } else {
        vx = (v1 + v2 + v3 + v4) / 4.0f;
        vy = 0.0f;
        w  = (v2 + v4 - v1 - v3) / (2.0f * wheel_distance_);
    }
}

// ================ 时间流转中心：脉冲 → 速度 → 里程计 ================

/**
 * @brief 每隔一定时间调用，喂入四个编码器的当前脉冲数值
 *
 * 这个函数做了 3 件事：
 *   1. 根据 Δ脉冲 / Δ时间 = 速度，算出四个轮子的真实物理速度
 *   2. 调用正运动学，把轮速变成车体速度
 *   3. 积分车体速度得到里程计位置（航位推算）
 */
void Kinematics::update_motor_ticks(uint64_t current_time,
                                     int32_t t1, int32_t t2,
                                     int32_t t3, int32_t t4) {
    // 计算时间间隔（微秒）
    uint32_t dt = current_time - motor_param_[0].last_update_time;

    static int32_t dticks[4];
    static int32_t motor_ticks[4];

    motor_ticks[0] = t1; motor_ticks[1] = t2;
    motor_ticks[2] = t3; motor_ticks[3] = t4;

    for (int i = 0; i < 4; i++) {
        // 这段时间内新增的脉冲数
        dticks[i] = motor_ticks[i] - motor_param_[i].last_encoder_tick;
        // 速度 = (Δ脉冲 × speed_factor) / Δ时间
        // speed_factor = 单脉冲距离(mm) × 10^6（把 us 转成 s）
        motor_param_[i].motor_speed = dticks[i] *
            (motor_param_[i].speed_factor / dt);
        // 更新历史，准备下个周期
        motor_param_[i].last_encoder_tick = motor_ticks[i];
        motor_param_[i].last_update_time = current_time;
    }

    // 正运动学 + 里程计积分
    this->update_bot_odom_(dt);
}

/**
 * @brief 航位推算 (Dead Reckoning)
 *
 * 核心思想：我知道 dt 秒前的位置和姿态，我也知道这 dt 秒内的速度
 *         → 用旋转矩阵把车体系速度投影到世界系 → 积分到位置
 *
 * 公式（车体系 → 世界系的旋转矩阵）：
 *   dx = (vx×cos(yaw) - vy×sin(yaw)) × dt
 *   dy = (vx×sin(yaw) + vy×cos(yaw)) × dt
 *   dyaw = w × dt
 */
void Kinematics::update_bot_odom_(uint32_t dt) {
    static float vx, vy, w;
    float dt_s = (float)(dt / 1000) / 1000;  // 微秒 → 秒

    // 正运动学：四个轮速 → 车体速度
    this->kinematic_forward(
        motor_param_[0].motor_speed, motor_param_[1].motor_speed,
        motor_param_[2].motor_speed, motor_param_[3].motor_speed,
        vx, vy, w);

    // 存入 Odom（mm/s → m/s）
    odom_.angular_speed = w;
    odom_.linear_x_speed = vx / 1000;
    odom_.linear_y_speed = vy / 1000;

    // 打滑补偿系数（实际跑多了之后实测标定）
    if (motion_motion_ == MOTION_OMNIDIRECTIONAL) {
        odom_.linear_x_speed *= calib_mx_;
        odom_.angular_speed   *= calib_myaw_;
    } else {
        odom_.linear_x_speed *= calib_dx_;
        odom_.angular_speed   *= calib_dyaw_;
    }

    // 【旋转矩阵】把车体速度投射到世界坐标系，积分得到绝对位置
    // 为什么不是简单的 +vx*dt？因为车体是旋转的，车体的"前方"在世界系里是 (cos(yaw), sin(yaw))
    odom_.x += (odom_.linear_x_speed * cos(odom_.yaw) -
                odom_.linear_y_speed * sin(odom_.yaw)) * dt_s;
    odom_.y += (odom_.linear_x_speed * sin(odom_.yaw) +
                odom_.linear_y_speed * cos(odom_.yaw)) * dt_s;
    odom_.yaw += odom_.angular_speed * dt_s;

    // 约束角度到 [-π, π]，防止转了 370 度变成很大的数导致 cos/sin 溢出
    Kinematics::TransAngleInPI(odom_.yaw, odom_.yaw);
}

odom_t &Kinematics::odom() {
    Euler2Quaternion(0, 0, odom_.yaw, odom_.quaternion);
    return odom_;
}
