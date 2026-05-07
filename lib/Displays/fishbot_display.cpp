/**
 * @file fishbot_display.cpp
 * @brief OLED 屏幕渲染引擎 — 三页切换 + GPIO0 按键
 *
 * 渲染节流设计：
 *   I2C 往 OLED 发 128x64=8192 像素的数据非常耗时（~10ms），
 *   如果用 100Hz 刷，会吃掉 100% CPU。所以用 1Hz 节流阀：
 *   即使外部每秒调用 updateDisplay() 一万次，屏幕也只每秒刷一次。
 *   但按键检测不受节流限制——它在 updateDisplay() 开头就跑了。
 */

#include "fishbot_display.h"

// ======================== 初始化 ========================

void FishBotDisplay::init() {
    // GPIO 0 配置为上拉输入（BOOT 按钮，按下为 LOW）
    pinMode(BTN_PIN, INPUT_PULLUP);
    btn_armed_ = true;   // 松手状态，允许触发
    display_mode_ = 0;   // 从状态页开始

    // I2C 总线初始化（SDA=12, SCL=13, 高速模式 400kHz）
    Wire.begin(12, 13, 400000UL);
    _display = Adafruit_SSD1306(128, 64, &Wire);
    _display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // 0x3C 是绝大多数 OLED 的 I2C 地址
    _display.clearDisplay();
    _display.setTextSize(1);
    _display.setTextColor(SSD1306_WHITE);
    _display.setCursor(0, 0);
    _display.print("    ");
    _display.println(version_code_);
    _display.println("");
    _display.println("system starting...");
    _display.display();

    memset(wheel_target_speed_, 0, sizeof(wheel_target_speed_));
    memset(wheel_actual_speed_, 0, sizeof(wheel_actual_speed_));
    memset(wheel_pwm_duty_, 0, sizeof(wheel_pwm_duty_));
}

FishBotDisplay::FishBotDisplay() {}

// ======================== 按键检测 ========================

/**
 * @brief GPIO 0 按键检测 — 按下即切页
 *
 * 状态机：
 *   松手(btn_armed_=true) → 按下 → 切页 + 解除武装(btn_armed_=false)
 *   → 按住不放也不切 → 松手 → 重新武装(btn_armed_=true) → 可以再次触发
 *
 * 这样设计的好处：
 *   - 按一下就切一页（不会像按住不放狂切）
 *   - 不需要消抖定时器（loop 的 100ms 间隔就是天然消抖）
 *   - 不需要边沿检测（松手才能再按即可靠保护）
 */
void FishBotDisplay::checkPageButton() {
    bool pressed = (digitalRead(BTN_PIN) == LOW);

    if (!pressed) {
        btn_armed_ = true;  // 松手了，重新武装
        return;
    }

    // 按下了并且已武装 → 切页
    if (btn_armed_) {
        display_mode_ = (display_mode_ + 1) % 3;  // 0→1→2→0 循环
        btn_armed_ = false;  // 解除武装，防止按住连切
        Serial.printf("[BTN] 切换页面 -> %d\n", display_mode_);
    }
}

// ======================== Page 0: 运行状态页 ========================

void FishBotDisplay::drawStatusPage() {
    String timenow = String(hour()) + ":" + twoDigits(minute()) +
                     ":" + twoDigits(second());
    _display.print("P0  ");
    _display.println(version_code_);
    _display.print("mode:");
    _display.println(mode_);

    if (mode_ == "udp_client") {
        if (wifi_status_ == FISHBOT_WIFI_STATUS_OK) {
            // WiFi + Agent 都 OK → 显示完整运行信息
            _display.print("time :"); _display.println(timenow);
            _display.print("ip:");    _display.println(wifi_ip_);
            _display.print("voltage :"); _display.println(battery_info_);
            _display.print("linear  :"); _display.println(bot_linear_);
            _display.print("angular :"); _display.println(bot_angular_);
        } else if (wifi_status_ == FISHBOT_WIFI_STATUS_PING_FAILED ||
                   wifi_status_ == FISHBOT_WIFI_STATUS_GOT_IP) {
            // WiFi 连上了但 Agent 不通 → 显示网络调试信息
            _display.print("wifi:");  _display.println(wifi_info_);
            _display.print("ip:");    _display.println(wifi_ip_);
            _display.print("ssid:");  _display.println(wifi_ssid_);
            _display.print("sip:");   _display.println(wifi_server_ip_);
        } else {
            // 还没连上 → 显示凭据方便排查
            _display.print("wifi:");  _display.println(wifi_info_);
            _display.print("ssid:");  _display.println(wifi_ssid_);
            _display.print("pswd:");  _display.println(wifi_pswd_);
        }
    } else {
        // 串口模式
        _display.print("time :");   _display.println(timenow);
        _display.print("motion:");  _display.println(motion_mode_);
        _display.print("baud :");   _display.println(baudrate_);
        _display.print("voltage :"); _display.println(battery_info_);
        _display.print("linear  :"); _display.println(bot_linear_);
        _display.print("angular :"); _display.println(bot_angular_);
    }
}

// ======================== Page 1: 调试数据页 ========================

void FishBotDisplay::drawDebugPage() {
    _display.println("P1 === MOTOR DEBUG ===");

    // 目标速度行（运动学解算出来的期望值）
    _display.print("T:");  // Target
    for (int i = 0; i < 4; i++) {
        _display.print(wheel_target_speed_[i], 0);
        if (i < 3) _display.print(" ");
    }
    _display.println();

    // 实际速度行（编码器实测值）
    _display.print("A:");  // Actual
    for (int i = 0; i < 4; i++) {
        _display.print(wheel_actual_speed_[i], 0);
        if (i < 3) _display.print(" ");
    }
    _display.println();

    // PWM 占空比行（PID 输出）
    _display.print("P:");  // PWM
    for (int i = 0; i < 4; i++) {
        _display.print(wheel_pwm_duty_[i], 0);
        if (i < 3) _display.print(" ");
    }
    _display.println();

    _display.print("Vx:"); _display.print(bot_linear_, 2);
    _display.print(" W:"); _display.println(bot_angular_, 2);
    _display.print("Bat:"); _display.print(battery_info_, 1);
    _display.println("V");
}

// ======================== Page 2: IMU 原始数据页 ========================

void FishBotDisplay::drawImuPage() {
    _display.println("P2 ==== IMU RAW ====");

    // 陀螺仪：三轴角速度 (°/s)
    _display.print("G:");  // Gyroscope
    _display.print(imu_gyro_x_, 1); _display.print(" ");
    _display.print(imu_gyro_y_, 1); _display.print(" ");
    _display.println(imu_gyro_z_, 1);

    // 加速度计：三轴线加速度 (m/s^2)
    _display.print("A:");  // Accelerometer
    _display.print(imu_acc_x_, 1); _display.print(" ");
    _display.print(imu_acc_y_, 1); _display.print(" ");
    _display.println(imu_acc_z_, 1);

    // 欧拉角：Roll/Pitch/Yaw (度)
    _display.print("R:"); _display.print(imu_euler_roll_, 1);
    _display.print(" P:"); _display.print(imu_euler_pitch_, 1);
    _display.println();
    _display.print("Y:"); _display.print(imu_euler_yaw_, 1);
    _display.print(" OK");
}

// ======================== 主渲染引擎 ========================

void FishBotDisplay::updateDisplay() {
    // 按键检测每帧都跑（不受下面的 1Hz 节流限制）
    checkPageButton();

    // 1Hz 节流阀：一秒只刷一次屏幕
    if (millis() - last_update_time < update_interval) return;
    last_update_time = millis();

    _display.clearDisplay();
    _display.setCursor(0, 0);

    switch (display_mode_) {
        case 0: drawStatusPage(); break;
        case 1: drawDebugPage();  break;
        case 2: drawImuPage();    break;
        default: drawStatusPage(); break;
    }

    _display.display();  // 真正把像素数据推到 OLED 硬件
}

// ======================== Setters ========================

void FishBotDisplay::updateVersionCode(String v)          { version_code_ = v; }
void FishBotDisplay::updateBatteryInfo(float &v)          { battery_info_ = v; }
void FishBotDisplay::updateUltrasoundDist(float &v)       { ultrasound_distance_ = v; }
void FishBotDisplay::updateBotAngular(float &v)           { bot_angular_ = v; }
void FishBotDisplay::updateBotLinear(float &v)            { bot_linear_ = v; }
void FishBotDisplay::updateTransMode(String m)            { mode_ = m; }
void FishBotDisplay::updateWIFIServerIp(String ip)        { wifi_server_ip_ = ip; }
void FishBotDisplay::updateWIFIIp(String ip)              { if (wifi_ip_ != ip) wifi_ip_ = ip; }
void FishBotDisplay::updateWIFIInfo(String i, fishbot_wifi_status_t s) {
    wifi_info_ = i; wifi_status_ = s;
}
void FishBotDisplay::updateCurrentTime(int64_t t)         { current_time = t; }
void FishBotDisplay::updateBaudRate(uint32_t b)           { baudrate_ = b; }
void FishBotDisplay::updateMotionMode(String m)           { motion_mode_ = m; }
void FishBotDisplay::updateDisplayMode(uint8_t m)         { display_mode_ = m; }
void FishBotDisplay::updateWIFISSID(String s)             { wifi_ssid_ = s; }
void FishBotDisplay::updateWIFIPSWD(String p)             { wifi_pswd_ = p; }

void FishBotDisplay::updateWheelDebug(const float t[4], const float a[4], const float p[4]) {
    memcpy(wheel_target_speed_, t, sizeof(wheel_target_speed_));
    memcpy(wheel_actual_speed_, a, sizeof(wheel_actual_speed_));
    memcpy(wheel_pwm_duty_,      p, sizeof(wheel_pwm_duty_));
}

void FishBotDisplay::updateImuRaw(float gx, float gy, float gz,
                                   float ax, float ay, float az,
                                   float roll, float pitch, float yaw) {
    imu_gyro_x_ = gx; imu_gyro_y_ = gy; imu_gyro_z_ = gz;
    imu_acc_x_  = ax;  imu_acc_y_  = ay;  imu_acc_z_ = az;
    imu_euler_roll_ = roll; imu_euler_pitch_ = pitch; imu_euler_yaw_ = yaw;
}

String FishBotDisplay::twoDigits(int digits) {
    if (digits < 10) return '0' + String(digits);
    return String(digits);
}

void FishBotDisplay::updateStartupInfo() {
    // 系统启动时显示的信息页（当前未使用，保留作为 API）
    String timenow = String(hour()) + ":" + twoDigits(minute()) +
                     ":" + twoDigits(second());
    last_update_time = millis();
    _display.clearDisplay();
    _display.setCursor(0, 0);
    _display.print("    "); _display.println(version_code_);
    _display.print("mode:"); _display.println(mode_);
    _display.print("voltage:"); _display.println(battery_info_);
    _display.println("");
    _display.println("system starting...");
    _display.display();
}
