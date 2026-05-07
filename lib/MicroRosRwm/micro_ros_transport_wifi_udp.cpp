#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <WiFi.h>
#include <WiFiUdp.h>

#include <uxr/client/util/time.h>
#include <uxr/client/profile/transport/custom/custom_transport.h>
#include "micro_ros_transport_wifi_udp.h"
extern "C"
{

  static WiFiUDP udp_client;   // 实例化一个极其核心的对象：Arduino 的 UDP 客户端硬件驱动

  bool platformio_transport_open_wifi_udp(struct uxrCustomTransport *transport)
  {
    //把地址本掏出来
    struct micro_ros_agent_locator *locator = (struct micro_ros_agent_locator *)transport->args;
    //开启ESP32底层的UDP监听端口
    return true == udp_client.begin(locator->port);
  }

  bool platformio_transport_close_wifi_udp(struct uxrCustomTransport *transport)
  {
    udp_client.stop();  //关掉网卡监听
    return true;
  }

  // 动作 3：发送数据 (Write) - UDP 是把数据打包装进一个个“包裹”里扔出去的
  size_t platformio_transport_write_wifi_udp(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode)
  {
    (void)errcode;
    struct micro_ros_agent_locator *locator = (struct micro_ros_agent_locator *)transport->args;

    size_t sent = 0;
    // 第一步：在包裹上写上收件人（电脑的 IP 和端口）
    if (true == udp_client.beginPacket(locator->address, locator->port))
    {
      //第二步：数据塞进包裹
      sent = udp_client.write(buf, len);
      //第三步：把包裹发送出去，发送成功保留已发送字节数，失败则归零
      sent = true == udp_client.endPacket() ? sent : 0;
    }

    //强行把网卡缓存里的数据推出去，不留残渣
    udp_client.flush();

    return sent;
  }

  size_t platformio_transport_read_wifi_udp(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
  {
    (void)errcode;
    // 记录开始等快递的时间 (uxr_millis 是 Micro-ROS 自带的毫秒时钟)
    int64_t start_time = uxr_millis();

    // 【带超时的等待死循环】
      // 条件 1：等的时间还没超过 timeout 毫秒
      // 条件 2：parsePacket() 检查网卡，返回 0 说明还没收到完整的 UDP 数据包
    while ((uxr_millis() - start_time) < ((int64_t)timeout) && udp_client.parsePacket() == 0)
    {
      delay(1);  //睡1ms，避免CPU卡死
    }

    size_t available = 0;
    // 循环结束了，赶紧看看是不是真的收到快递了
    if (udp_client.available())
    {
      available = udp_client.read(buf, len);
    }

    return (available > 0) ? available : 0;
  }
}