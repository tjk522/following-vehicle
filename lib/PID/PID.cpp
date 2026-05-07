#include "PID.h"

// 构造函数：初始化参数、限幅并强制重置状态
PidController::PidController(float kp, float ki, float kd, float out_min, float out_max) {
    update_pid(kp, ki, kd);
    out_limit(out_min, out_max);
    reset();
}

// 核心计算律：增量式 PID
float PidController::update(float current_measure) {
    // 1. 计算当前误差 e(k)=目标值 - 反馈值
    float error = target_ - current_measure;

    // 2. 严格遵循离散化物理方程计算控制增量 delta_u
    // 比例项增量: Kp * [e(k) - e(k-1)]
    // 积分项增量: Ki * e(k)
    // 微分项增量: Kd * [e(k) - 2e(k-1) + e(k-2)]
    float delta_u = kp_ * (error - error_prev_) +
                    ki_ * error +
                    kd_ * (error - 2.0f * error_prev_ + error_prev2_);

    // 3. 累加得到本次的绝对输出量 u(k) = u(k-1) + delta_u
    pwm_output_ += delta_u;

    // 4. 输出限幅 (保护电机驱动板物理上限)
    if (pwm_output_ > out_max_) {
        pwm_output_ = out_max_;
    } else if (pwm_output_ < out_min_) {
        pwm_output_ = out_min_;
    }

    // 5. 严格递推历史状态，准备下个控制周期
    error_prev2_ = error_prev_;
    error_prev_  = error;

    return pwm_output_;
}

// 更新目标值
void PidController::update_target(float target) {
    target_ = target;
}

// 在线动态调整 PID 参数
void PidController::update_pid(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

// 设置输出限幅
void PidController::out_limit(float out_min, float out_max) {
    out_min_ = out_min;
    out_max_ = out_max;
}

// 彻底清空历史状态 (在电机急停、重新启动时调用，防止历史误差带来冲击)
void PidController::reset() {
    target_ = 0.0f;
    error_prev_ = 0.0f;
    error_prev2_ = 0.0f;
    pwm_output_ = 0.0f;
}