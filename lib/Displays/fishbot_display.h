#ifndef __FISHBOT_DISPLAY_H__
#define __FISHBOT_DISPLAY_H__
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TimeLib.h>

enum fishbot_wifi_status_t
{
    FISHBOT_WIFI_STATUS_OK,
    FISHBOT_WIFI_STATUS_NO_FOUND,
    FISHBOT_WIFI_STATUS_PASD_ERROR,
    FISHBOT_WIFI_STATUS_WAIT_CONNECT,
    FISHBOT_WIFI_STATUS_PING_FAILED,
    FISHBOT_WIFI_STATUS_GOT_IP,
    FISHBOT_WIFI_STATUS_UNKNOW,
};

class FishBotDisplay
{
private:
    Adafruit_SSD1306 _display;

    // --- 运行状态页数据 ---
    float battery_info_;
    float ultrasound_distance_;
    float bot_angular_;
    float bot_linear_;
    uint32_t baudrate_;
    String mode_;
    String version_code_;
    uint8_t display_mode_;       // 0=状态页 1=调试页 2=IMU页

    // --- 显示控制 ---
    int64_t current_time;
    uint64_t last_update_time;
    uint64_t update_interval{1000};

    // --- WiFi 数据 ---
    String motion_mode_;
    String wifi_ssid_;
    String wifi_pswd_;
    String wifi_ip_;
    String wifi_server_ip_;
    String wifi_info_ = "wait connect";
    fishbot_wifi_status_t wifi_status_ = FISHBOT_WIFI_STATUS_WAIT_CONNECT;

    // --- 调试页数据 ---
    float wheel_target_speed_[4];
    float wheel_actual_speed_[4];
    float wheel_pwm_duty_[4];

    // --- IMU 原始数据 ---
    float imu_gyro_x_, imu_gyro_y_, imu_gyro_z_;
    float imu_acc_x_, imu_acc_y_, imu_acc_z_;
    float imu_euler_roll_, imu_euler_pitch_, imu_euler_yaw_;

    // --- GPIO0 按键切换页面 ---
    static const uint8_t BTN_PIN = 0;
    bool btn_armed_;

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
