/*
 * @Author: lalala123 1054060225@qq.com
 * @Date: 2023-03-23 17:23:07
 * @LastEditors: lalala123 1054060225@qq.com
 * @LastEditTime: 2023-04-02 00:29:42
 * @FilePath: \fishbot_motion_control_microros\lib\MicroRosRwm\micro_ros_transport_wifi_udp.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include <WiFi.h>
#include <WiFiUdp.h>
#include "micro_ros_platformio.h"

extern "C"
{
    bool platformio_transport_open_wifi_udp(struct uxrCustomTransport *transport);
    bool platformio_transport_close_wifi_udp(struct uxrCustomTransport *transport);
    size_t platformio_transport_write_wifi_udp(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
    size_t platformio_transport_read_wifi_udp(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);
}

// 这个结构体是给 UDP 用的“目标地址本”。里面记录了你的电脑（Agent）的 IP 和 端口
struct micro_ros_agent_locator
{
    IPAddress address;
    int port;
};
// 用于设置 Micro-ROS 的 WiFi 通信传输方式的函数。
// 具体实现如下：传入 WiFi 的 SSID、密码、代理 IP 地址、代理端口号和设备名称。
static bool set_microros_wifi_transports(const char *ssid, const char *pswd, IPAddress agent_ip, uint16_t agent_port, String device_name)
{
    if (!WiFi.setHostname(device_name.c_str()))
    {
        Serial.println("Hostname failed to configure");
    }
    // 以 Station 模式连接到 WiFi 网络
    WiFi.mode(WIFI_STA);
    // 连接到指定的 WiFi 网络
    WiFi.begin(ssid, pswd);
    // 开启静态缓冲区，以便更好地管理内存
    // WiFi 底层收发数据包极其频繁。如果不加这句，系统会疯狂 malloc 和 free 动态内存，
    // ESP32 跑半天就会因为“内存碎片化”彻底死机。开启后，系统会提前划一块固定地盘给 WiFi 用，极其稳定。
    WiFi.useStaticBuffers(true);
    // 开启自动重连功能
    WiFi.setAutoReconnect(true);

    // 进行 WiFi 连接，最多尝试 4 次，每次间隔 500 毫秒。如果 WiFi 连接失败，则输出错误信息
    for (int i = 0; i < 4; i++)
    {
        if (WiFi.status() != WL_CONNECTED)
        {
            delay(500);
        }
        else
        {
            break;
        }
    }
    if (WiFi.status() != WL_CONNECTED)
    {
        Serial.println("wifi connected failed!");
    }
    // 不使用睡眠模式
    // ESP32 为了省电，默认在没有数据时会让天线“打盹”。
    // 这会导致延迟忽高忽低，Micro-ROS 电脑端会认为小车网络极差，甚至判定小车掉线！
    // WIFI_PS_NONE 强行命令天线 100% 功率永远睁着眼睛，这是做机器人的标配。
    WiFi.setSleep(WIFI_PS_NONE);
    // 创建一个结构体类型的变量 locator，用于存储代理 IP 地址和端口号
    static struct micro_ros_agent_locator locator;
    locator.address = agent_ip;
    locator.port = agent_port;
    // 调用 rmw_uros_set_custom_transport 函数，将自定义的 WiFi 传输函数注册到 Micro-ROS 中
    // 注意第一个参数变成了 false，告诉 Micro-ROS：
    // “UDP 是以打包形式发数据的（Datagram），和串口的流式（Stream）不一样，注意丢包处理！”
    rmw_uros_set_custom_transport(
        false,
        (void *)&locator,
        platformio_transport_open_wifi_udp,
        platformio_transport_close_wifi_udp,
        platformio_transport_write_wifi_udp,
        platformio_transport_read_wifi_udp);
    return true;
}