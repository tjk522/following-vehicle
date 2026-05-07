#include <Arduino.h>

#include <micro_ros_platformio.h>

#include <stdio.h>
#include <stdbool.h>
#include <sys/time.h>

#include <uxr/client/profile/transport/custom/custom_transport.h>
#include "micro_ros_transport_serial.h"

// 【核心语法：extern "C"】
// 为什么要把函数包在这里面？
// 因为 Micro-ROS 的底层是纯 C 语言编译的，而 Arduino 框架是 C++ 编译的。
// C++ 编译器会对函数名进行“重整（Name Mangling）”，导致 C 语言找不到这些函数。
// extern "C" 就是告诉编译器：“这几个函数给我按老式的 C 语言规则编译，方便 Micro-ROS 来调用！”
extern "C"
{
  // 动作 1：打开管道 (Open)
    // 对于串口来说，我们在 main.cpp 里的 setup() 早就用 Serial.begin(115200) 打开了，
    // 所以这里什么都不用干，直接骗 Micro-ROS 说：“已经打开啦！”返回 true。
  bool platformio_transport_open_serial(struct uxrCustomTransport *transport)
  {
    return true;
  }

  // 动作 2：关闭管道 (Close)
    // 同理，单片机的串口一般不关闭，直接返回 true。
  bool platformio_transport_close_serial(struct uxrCustomTransport *transport)
  {
    return true;
  }

  // 动作 3：发送数据 (Write)
    // 当 Micro-ROS 想要把 Odom 数据发给电脑时，它会调用这个函数，并把数据放进 buf 数组里。
  size_t platformio_transport_write_serial(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *errcode)
  {
    (void)errcode;  // 忽略错误码，防止编译器报“变量未使用”的警告
    // transport->args 里存着咱们传进来的串口对象（比如 Serial）。
    // 把它强制转换回 Arduino 的 Stream 类指针。
    Stream *stream = (Stream *)transport->args;
    // 调用 Arduino 底层的串口发送函数，把 buf 里的数据全部射出去！
    // 返回成功发送的字节数给 Micro-ROS。
    size_t sent = stream->write(buf, len);
    return sent;
  }

  // 动作 4：接收数据 (Read)
    // 当 Micro-ROS 想看看有没有电脑发来的速度指令时，它会调用这个函数。
  size_t platformio_transport_read_serial(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode)
  {
    (void)errcode;

    Stream *stream = (Stream *)transport->args;
    stream->setTimeout(timeout);   // 设置等待超时时间。如果电脑一直没发数据，不能让单片机遇在这里死等，等 timeout 毫秒后必须放弃。
    return stream->readBytes((char *)buf, len);
  }
}