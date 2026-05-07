**麦克纳姆轮运动学**
*运动学逆解*
void Kinematics::kinematic_inverse(float linear_x_speed, float linear_y_speed, float angular_speed,
                                   float &out_wheel_speed1, float &out_wheel_speed2, float &out_wheel_speed3, float &out_wheel_speed4)
{
    const float a = 108.0f;  // 机器人中心到轮子沿x方向的距离
    const float b = 88.5f;   // 机器人中心到轮子沿y方向的距离

    out_wheel_speed1 = linear_x_speed - linear_y_speed - angular_speed * (a + b);
    out_wheel_speed2 = linear_x_speed + linear_y_speed + angular_speed * (a + b);
    out_wheel_speed3 = linear_x_speed + linear_y_speed - angular_speed * (a + b);
    out_wheel_speed4 = linear_x_speed - linear_y_speed + angular_speed * (a + b);

    // Debug 输出
    // Serial.printf("out_wheel_speed[%f,%f,%f,%f]\n", out_wheel_speed1, out_wheel_speed2, out_wheel_speed3, out_wheel_speed4);
}

*运动学正解*
void Kinematics::kinematic_forward(float wheel1_speed, float wheel2_speed, float wheel3_speed, float wheel4_speed,
                                   float &linear_x_speed, float &linear_y_speed, float &angular_speed)
{
    const float a = 108.0f;  // 机器人中心到轮子沿x方向的距离
    const float b = 88.5f;   // 机器人中心到轮子沿y方向的距离

    // 计算机器人线速度和角速度
    linear_x_speed = (wheel1_speed + wheel2_speed + wheel3_speed + wheel4_speed) / 4.0f;
    linear_y_speed = (-wheel1_speed + wheel2_speed + wheel3_speed - wheel4_speed) / 4.0f;
    angular_speed = float(-wheel1_speed + wheel2_speed - wheel3_speed + wheel4_speed) / (4.0f * (a + b));

    // Debug 输出
    // Serial.printf("angular_speed:%f wheel_speed[%f,%f,%f,%f]\n",angular_speed, wheel1_speed, wheel2_speed, wheel3_speed, wheel4_speed);
}



但在地上带负载跑的时候，最高转速通常会打个八折，算作 100 RPM（即每秒转 1.66 圈）。结合轮子周长 150.8 mm：
小车绝对最高物理线速度  1.66 *150.8 =250 mm/s （即 0.25 m/s）


GlobalData.h
数据撕裂：Core 0 正在往 global_odom 里写 X 和 Y，刚写完 X，还没来得及写 Y，Core 1 突然把数据读走了。这时 Core 1 拿到的就是一个“一半新、一半旧”的“怪物数据”，会导致坐标瞬间飞移。所以必须引入 FreeRTOS 的自旋锁（Spinlock）。

通信超时：如果你的电脑/大模型程序崩了，不再下发 cmd_vel，如果下位机不知道，就会一直保持最后一次收到的速度疯狂前冲（俗称“疯车”）。所以必须加入时间戳（Timestamp）作为底层看门狗机制。

*关闭 VS Code 里的 PIO 自动更新*
让它开机时彻底闭嘴，再也不去偷偷查更新。

在 VS Code 里，按下快捷键 Ctrl + ,（逗号）打开设置。

在顶部的搜索框输入：platformio auto update



*头文件有红线*
强制重建 IntelliSense 索引（最推荐）
这是 PlatformIO 专门对付这个问题的绝招。

在 VS Code 里，按下快捷键 Ctrl + Shift + P （Mac 是 Cmd + Shift + P）调出顶部命令面板。

输入 Rebuild。

在下拉列表里找到并点击：PlatformIO: Rebuild IntelliSense Index。

等待右下角进度条跑完（大概几秒钟），你会看到红线像变魔术一样瞬间全部消失！

*set_microros_wifi_transports有红线*
// 【新增这一行！】引入你自己的 UDP 底层网卡驱动
#include <micro_ros_transport_wifi_udp.h>

freeRTOS中
delay会阻塞整个系统，破坏实时性，浪费CPU资源
vTaskDelay
vTaskDelayUntil

*连接小车*
sudo docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm --privileged --net=host registry.cn-hangzhou.aliyuncs.com/fishros/micro-ros-agent:$ROS_DISTRO udp4 --port 8888 -v6
111