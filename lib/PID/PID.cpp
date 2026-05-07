/**
 * @file PID.cpp
 * @brief 增量式 PID 控制器 — 机器人控制的核心算法
 *
 * PID 是什么？
 *   P(比例-Proportional)：误差越大输出越大，负责"快速接近目标"
 *   I(积分-Integral)：累积历史误差，负责"消除稳态偏差"（比如上坡时补力）
 *   D(微分-Derivative)：预测未来趋势，负责"抑制震荡"（刹车防过冲）
 *
 * 增量式 vs 位置式：
 *   增量式：Δu = Kp(eₖ-eₖ₋₁) + Ki·eₖ + Kd(eₖ-2eₖ₋₁+eₖ₋₂)
 *     优点：天然无积分饱和、输出平滑、切换目标不会突变
 *   位置式：u = Kp·eₖ + Ki·Σe + Kd(eₖ-eₖ₋₁)
 *     缺点：积分项会无限累积，需要额外的抗饱和处理
 *
 * 这里用增量式 — 最适合电机速度控制的场景。
 */

#include "PID.h"

PidController::PidController(float kp, float ki, float kd,
                             float out_min, float out_max) {
    update_pid(kp, ki, kd);
    out_limit(out_min, out_max);
    reset();
}

/**
 * @brief 核心计算：输入当前实际速度，输出 PWM 控制值
 * @param current_measure 编码器实测的当前轮速 (mm/s)
 * @return PWM 占空比值（-100 ~ 100）
 */
float PidController::update(float current_measure) {
    // 1. 计算本次误差 e(k) = 目标 - 实际
    float error = target_ - current_measure;

    // 2. 增量式 PID 公式（离散化推导）：
    //    Δu = Kp × [e(k) - e(k-1)]      ← 比例增量（"你还在错，我继续加力"）
    //       + Ki × e(k)                  ← 积分增量（"你一直错，我多出点力"）
    //       + Kd × [e(k) - 2e(k-1) + e(k-2)]  ← 微分增量（"你变得太快了，我刹车"）
    float delta_u = kp_ * (error - error_prev_) +
                    ki_ * error +
                    kd_ * (error - 2.0f * error_prev_ + error_prev2_);

    // 3. 累加得到最终输出 u(k) = u(k-1) + Δu
    pwm_output_ += delta_u;

    // 4. 输出限幅（保护电机驱动板，PWM 范围 -100~100）
    if (pwm_output_ > out_max_) pwm_output_ = out_max_;
    else if (pwm_output_ < out_min_) pwm_output_ = out_min_;

    // 5. 递推历史状态，为下个控制周期准备
    error_prev2_ = error_prev_;  // e(k-2) ← e(k-1)
    error_prev_  = error;         // e(k-1) ← e(k)

    return pwm_output_;
}

void PidController::update_target(float target) { target_ = target; }
void PidController::update_pid(float kp, float ki, float kd) {
    kp_ = kp; ki_ = ki; kd_ = kd;
}
void PidController::out_limit(float out_min, float out_max) {
    out_min_ = out_min; out_max_ = out_max;
}

/**
 * @brief 重置 PID 历史状态
 *
 * 什么时候调用？
 *   看门狗触发刹车时 — 不清历史的话，积分项会让松刹车瞬间猛冲
 *   电机急停重新启动时 — 避免历史误差累积产生冲击
 */
void PidController::reset() {
    target_ = 0.0f;
    error_prev_ = 0.0f;
    error_prev2_ = 0.0f;
    pwm_output_ = 0.0f;
}
