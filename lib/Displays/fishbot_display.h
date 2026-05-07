/**
 * @file fishbot_display.h
 * @brief OLED 屏幕驱动类（SSD1306 128x64 I2C）
 *
 * 128x64 像素，text size=1 时每字符 6x8 像素
 * → 128/6 ≈ 21 字符/行，64/8 = 8 行
 *
 * 三页切换架构：
 *   Page 0 (P0): 运行状态 — WiFi / IP / 电压 / 速度
 *   Page 1 (P1): 调试数据 — 四轮目标/实际/PWM
 *   Page 2 (P2): IMU 原始数据 — 陀螺仪 / 加速度 / 欧拉角
 *
 * GPIO 0 (BOOT) 按键切换：按下即切页，松手才能再切
 * 1Hz 渲染节流：I2C 刷屏很耗时，高频刷会卡底盘控制
 */
#ifndef __FISHBOT_DISPLAY_H__
#define __FISHBOT_DISPLAY_H__

#include <Wire.h>
#include <Adafruit_GFX.h>      // Adafruit 图形核心库（画点、线、字）
#include <Adafruit_SSD1306.h>  // SSD1306 OLED 底层驱动
#include <TimeLib.h>           // 时间格式化（hour():minute():second()）

// WiFi 的 7 种状态
enum fishbot_wifi_status_t {
    FISHBOT_WIFI_STATUS_OK,            // 0: 一切正常，Agent 也连上了
    FISHBOT_WIFI_STATUS_NO_FOUND,      // 1: 找不到 WiFi（SSID 错了 / 没开）
    FISHBOT_WIFI_STATUS_PASD_ERROR,    // 2: 密码错误
    FISHBOT_WIFI_STATUS_WAIT_CONNECT,  // 3: 正在连接...
    FISHBOT_WIFI_STATUS_PING_FAILED,   // 4: WiFi 连上了但 Ping 不通 Agent
    FISHBOT_WIFI_STATUS_GOT_IP,        // 5: 拿到 IP 了，还没连 Agent
    FISHBOT_WIFI_STATUS_UNKNOW,        // 6: 未知错误
};

class FishBotDisplay {
private:
    Adafruit_SSD1306 _display;

    // ---- Page 0: 运行状态数据 ----
    float battery_info_;
    float ultrasound_distance_;
    float bot_angular_;
    float bot_linear_;
    uint32_t baudrate_;
    String mode_;
    String version_code_;
    uint8_t display_mode_;  // 0=状态页 1=调试页 2=IMU页

    // ---- 显示控制 ----
    int64_t  current_time;
    uint64_t last_update_time;
    uint64_t update_interval{1000};  // 屏幕刷新间隔 1000ms = 1Hz

    // ---- WiFi 数据 ----
    String motion_mode_, wifi_ssid_, wifi_pswd_, wifi_ip_, wifi_server_ip_;
    String wifi_info_ = "wait connect";
    fishbot_wifi_status_t wifi_status_ = FISHBOT_WIFI_STATUS_WAIT_CONNECT;

    // ---- Page 1: 调试页数据 ----
    float wheel_target_speed_[4];
    float wheel_actual_speed_[4];
    float wheel_pwm_duty_[4];

    // ---- Page 2: IMU 原始数据 ----
    float imu_gyro_x_, imu_gyro_y_, imu_gyro_z_;
    float imu_acc_x_,  imu_acc_y_,  imu_acc_z_;
    float imu_euler_roll_, imu_euler_pitch_, imu_euler_yaw_;

    // ---- GPIO 0 按键切换 ----
    static const uint8_t BTN_PIN = 0;  // BOOT 按钮
    bool btn_armed_;  // 松手了吗？（防止按住时连续切页）

    // 三个页面的绘制函数
    void drawStatusPage();
    void drawDebugPage();
    void drawImuPage();
    void checkPageButton();

public:
    void init();
    void updateDisplayMode(uint8_t display_mode);
    void updateDisplay();
    void updateStartupInfo();
    void updateBatteryInfo(float &battery_info);
    void updateUltrasoundDist(float &ultrasound_distance);
    void updateBotAngular(float &bot_angular);
    void updateBotLinear(float &bot_linear);
    void updateTransMode(String mode);
    void updateCurrentTime(int64_t current_time_);
    void updateBaudRate(uint32_t baudrate);
    void updateWIFIIp(String ip);
    void updateWIFIServerIp(String server_ip);
    void updateWIFIInfo(String info, fishbot_wifi_status_t status);
    void updateWIFISSID(String ssid);
    void updateWIFIPSWD(String pswd);
    void updateVersionCode(String version_code);
    void updateMotionMode(String mode);
    void updateWheelDebug(const float target[4], const float actual[4], const float pwm[4]);
    void updateImuRaw(float gx, float gy, float gz,
                      float ax, float ay, float az,
                      float roll, float pitch, float yaw);
    String twoDigits(int digits);
    FishBotDisplay();
    ~FishBotDisplay() = default;
};

#endif
