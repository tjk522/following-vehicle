#include "fishbot_display.h"

void FishBotDisplay::init()
{
    pinMode(BTN_PIN, INPUT_PULLUP);
    btn_armed_ = true;
    display_mode_ = 0;

    Wire.begin(12, 13, 400000UL);
    _display = Adafruit_SSD1306(128, 64, &Wire);
    _display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
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

FishBotDisplay::FishBotDisplay()
{
}

// =====================================================================
// GPIO 0 按键切换页面: 按下即切，松手后才能再次触发
// =====================================================================
void FishBotDisplay::checkPageButton() {
    bool pressed = (digitalRead(BTN_PIN) == LOW);

    if (!pressed) {
        btn_armed_ = true;
        return;
    }

    if (btn_armed_) {
        display_mode_ = (display_mode_ + 1) % 3;
        btn_armed_ = false;
        Serial.printf("[BTN] 切换页面 -> %d\n", display_mode_);
    }
}

// =====================================================================
// Page 0: 运行状态页
// =====================================================================
void FishBotDisplay::drawStatusPage()
{
    String timenow = String(hour()) + ":" + twoDigits(minute()) + ":" + twoDigits(second());
    _display.print("P0  ");
    _display.println(version_code_);
    _display.print("mode:");
    _display.println(mode_);

    if (mode_ == "udp_client")
    {
        if (wifi_status_ == FISHBOT_WIFI_STATUS_OK)
        {
            _display.print("time :");
            _display.println(timenow);
            _display.print("ip:");
            _display.println(wifi_ip_);
            _display.print("voltage :");
            _display.println(battery_info_);
            _display.print("linear  :");
            _display.println(bot_linear_);
            _display.print("angular :");
            _display.println(bot_angular_);
        }
        else if (wifi_status_ == FISHBOT_WIFI_STATUS_PING_FAILED || wifi_status_ == FISHBOT_WIFI_STATUS_GOT_IP)
        {
            _display.print("wifi:");
            _display.println(wifi_info_);
            _display.print("ip:");
            _display.println(wifi_ip_);
            _display.print("ssid:");
            _display.println(wifi_ssid_);
            _display.print("sip:");
            _display.println(wifi_server_ip_);
        }
        else
        {
            _display.print("wifi:");
            _display.println(wifi_info_);
            _display.print("ssid:");
            _display.println(wifi_ssid_);
            _display.print("pswd:");
            _display.println(wifi_pswd_);
        }
    }
    else
    {
        _display.print("time :");
        _display.println(timenow);
        _display.print("motion:");
        _display.println(motion_mode_);
        _display.print("baud :");
        _display.println(baudrate_);
        _display.print("voltage :");
        _display.println(battery_info_);
        _display.print("linear  :");
        _display.println(bot_linear_);
        _display.print("angular :");
        _display.println(bot_angular_);
    }
}

// =====================================================================
// Page 1: 调试数据页 — 四轮目标/实际/PWM
// =====================================================================
void FishBotDisplay::drawDebugPage()
{
    _display.println("P1 === MOTOR DEBUG ===");

    // 目标速度行
    _display.print("T:");
    for (int i = 0; i < 4; i++) {
        _display.print(wheel_target_speed_[i], 0);
        if (i < 3) _display.print(" ");
    }
    _display.println();

    // 实际速度行
    _display.print("A:");
    for (int i = 0; i < 4; i++) {
        _display.print(wheel_actual_speed_[i], 0);
        if (i < 3) _display.print(" ");
    }
    _display.println();

    // PWM 占空比行
    _display.print("P:");
    for (int i = 0; i < 4; i++) {
        _display.print(wheel_pwm_duty_[i], 0);
        if (i < 3) _display.print(" ");
    }
    _display.println();

    // 线速度/角速度摘要
    _display.print("Vx:");
    _display.print(bot_linear_, 2);
    _display.print(" W:");
    _display.println(bot_angular_, 2);

    _display.print("Bat:");
    _display.print(battery_info_, 1);
    _display.println("V");
}

// =====================================================================
// Page 2: IMU 原始数据页
// =====================================================================
void FishBotDisplay::drawImuPage()
{
    _display.println("P2 ==== IMU RAW ====");

    // 陀螺仪 (角速度 °/s)
    _display.print("G:");
    _display.print(imu_gyro_x_, 1);
    _display.print(" ");
    _display.print(imu_gyro_y_, 1);
    _display.print(" ");
    _display.println(imu_gyro_z_, 1);

    // 加速度计 (m/s^2)
    _display.print("A:");
    _display.print(imu_acc_x_, 1);
    _display.print(" ");
    _display.print(imu_acc_y_, 1);
    _display.print(" ");
    _display.println(imu_acc_z_, 1);

    // 欧拉角 (度)
    _display.print("R:");
    _display.print(imu_euler_roll_, 1);
    _display.print(" P:");
    _display.print(imu_euler_pitch_, 1);
    _display.println();

    _display.print("Y:");
    _display.print(imu_euler_yaw_, 1);

    // IMU 是否正常
    _display.print(" OK");
}

// =====================================================================
// 主渲染引擎
// =====================================================================
void FishBotDisplay::updateDisplay()
{
    // 按键检测每帧都跑，不受显示刷新率限制
    checkPageButton();

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

    _display.display();
}

// ======================== Setters ========================

void FishBotDisplay::updateVersionCode(String version_code) { version_code_ = version_code; }
void FishBotDisplay::updateBatteryInfo(float &battery_info) { battery_info_ = battery_info; }
void FishBotDisplay::updateUltrasoundDist(float &ultrasound_distance) { ultrasound_distance_ = ultrasound_distance; }
void FishBotDisplay::updateBotAngular(float &bot_angular) { bot_angular_ = bot_angular; }
void FishBotDisplay::updateBotLinear(float &bot_linear) { bot_linear_ = bot_linear; }
void FishBotDisplay::updateTransMode(String mode) { mode_ = mode; }
void FishBotDisplay::updateWIFIServerIp(String server_ip) { wifi_server_ip_ = server_ip; }
void FishBotDisplay::updateWIFIIp(String ip) { if (wifi_ip_ != ip) wifi_ip_ = ip; }
void FishBotDisplay::updateWIFIInfo(String info, fishbot_wifi_status_t status) { wifi_info_ = info; wifi_status_ = status; }
void FishBotDisplay::updateCurrentTime(int64_t current_time_) { current_time = current_time_; }

String FishBotDisplay::twoDigits(int digits) {
    if (digits < 10) return '0' + String(digits);
    return String(digits);
}

void FishBotDisplay::updateBaudRate(uint32_t baudrate) { baudrate_ = baudrate; }
void FishBotDisplay::updateMotionMode(String mode) { motion_mode_ = mode; }

void FishBotDisplay::updateStartupInfo() {
    String timenow = String(hour()) + ":" + twoDigits(minute()) + ":" + twoDigits(second());
    last_update_time = millis();
    _display.clearDisplay();
    _display.setCursor(0, 0);
    _display.print("    ");
    _display.println(version_code_);
    _display.print("mode:");
    _display.println(mode_);
    _display.print("voltage:");
    _display.println(battery_info_);
    _display.println("");
    _display.println("syetem starting...");
    _display.display();
}

void FishBotDisplay::updateDisplayMode(uint8_t display_mode) { display_mode_ = display_mode; }

void FishBotDisplay::updateWIFISSID(String ssid) { wifi_ssid_ = ssid; }
void FishBotDisplay::updateWIFIPSWD(String pswd) { wifi_pswd_ = pswd; }

void FishBotDisplay::updateWheelDebug(const float target[4], const float actual[4], const float pwm[4]) {
    memcpy(wheel_target_speed_, target, sizeof(wheel_target_speed_));
    memcpy(wheel_actual_speed_, actual, sizeof(wheel_actual_speed_));
    memcpy(wheel_pwm_duty_, pwm, sizeof(wheel_pwm_duty_));
}

void FishBotDisplay::updateImuRaw(float gx, float gy, float gz,
                                   float ax, float ay, float az,
                                   float roll, float pitch, float yaw) {
    imu_gyro_x_ = gx; imu_gyro_y_ = gy; imu_gyro_z_ = gz;
    imu_acc_x_ = ax;  imu_acc_y_ = ay;  imu_acc_z_ = az;
    imu_euler_roll_ = roll; imu_euler_pitch_ = pitch; imu_euler_yaw_ = yaw;
}
