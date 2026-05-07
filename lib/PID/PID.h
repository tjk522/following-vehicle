#ifndef PID_H
#define PID_H

class PidController {
private:
    // --- 控制参数 ---
    float kp_;
    float ki_;
    float kd_;

    // --- 物理状态变量 (严格递推，受 private 保护) ---
    float target_;        // 目标速度
    float error_prev_;    // e(k-1): 上一次的误差
    float error_prev2_;   // e(k-2): 上上一次的误差
    float pwm_output_;    // u(k): 当前计算出的控制绝对输出量

    // --- 输出限幅 ---
    float out_min_;
    float out_max_;

public:
    // --- 构造与初始化 ---
    PidController() = default;
    PidController(float kp, float ki, float kd, float out_min, float out_max);

    // --- 核心控制算法 ---
    // 传入当前反馈的实际速度，返回计算后的 PWM 驱动值
    float update(float current_measure);

    // --- 状态与参数管理 ---
    void reset();
    void update_pid(float kp, float ki, float kd);
    void update_target(float target);
    void out_limit(float out_min, float out_max);

    // --- 调试接口 ---
    // 方便在命令行纯数值打印调参使用
    float get_target() const { return target_; }
    float get_output() const { return pwm_output_; }
};

#endif