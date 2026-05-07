/**
 * @file MicroROS_Node.cpp
 * @brief Micro-ROS 通信节点 — WiFi + Agent + 话题发布订阅（Core 0）
 *
 * Micro-ROS 是什么？
 *   标准 ROS 2 跑在 Linux 上，需要几百 MB 内存。Micro-ROS 是给 MCU（ESP32）
 *   用的精简版 ROS 2，只有几十 KB，通过 UDP 和电脑上的 Micro-ROS Agent 通信。
 *
 * 通信架构：
 *   ESP32 (micro-ROS client) ←UDP WiFi→ 电脑 (micro-ROS Agent Docker) ←→ ROS 2 网络
 *
 * 本节点的话题：
 *   发布 (Publishers):
 *     /odom  - 里程计 (nav_msgs/Odometry, 20Hz)
 *     /imu   - IMU 数据 (sensor_msgs/Imu, 20Hz)
 *     /scan  - 激光雷达 (sensor_msgs/LaserScan, 每圈一发)
 *
 *   订阅 (Subscriptions):
 *     /cmd_vel        - 接收速度指令 (geometry_msgs/Twist)
 *     /leader_follow  - 跟随模式开关 (std_msgs/Bool)
 *
 * Executor 与 Handle：
 *   micro-ROS 用 Executor 管理多个订阅/定时器。handle 数 = 订阅数 + 定时器数。
 *   这里：2 订阅 + 1 定时器 = 3 handles。
 *   发布者不需要 handle（发布是主动调用的，不由 Executor 管理）。
 */

#include "MicroROS_Node.h"
#include "WebConfig.h"

// ================ WiFi / Agent 配置 ================
// 优先从 NVS 加载保存的凭据，没保存则用默认值
char g_wifi_ssid[32] = {0};
char g_wifi_pass[32] = {0};
IPAddress agent_ip(172, 20, 10, 3);  // 电脑局域网 IP
const uint32_t agent_port = 8888;     // Agent 监听端口

// ================ ROS 实体 ================
// 发布者（主动发数据给上位机）
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t laser_publisher;

// 订阅者（接收上位机指令）
rcl_subscription_t twist_subscriber;   // /cmd_vel
rcl_subscription_t follow_subscriber;  // /leader_follow

// ROS 消息对象（序列化前的数据结构）
nav_msgs__msg__Odometry     odom_msg;
sensor_msgs__msg__Imu       imu_msg;
sensor_msgs__msg__LaserScan laser_msg;
geometry_msgs__msg__Twist   twist_msg;
std_msgs__msg__Bool         follow_msg;

// Executor（管理订阅和定时器的事件循环"大管家"）
rclc_executor_t executor;
rclc_support_t   support;
rcl_allocator_t  allocator;
rcl_node_t       node;
rcl_timer_t      timer;  // 20Hz 定时器，触发 odom/imu/scan 的发布

// ================ Leader-Follower 开关状态 ================
bool leader_follow_enabled = false;

// =====================================================================
// 订阅回调：/cmd_vel → 写入 global_cmd（Core 0 → Core 1）
// =====================================================================
void twist_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg =
        (const geometry_msgs__msg__Twist *)msgin;

    // 抢锁 → 写目标速度 + 打时间戳 → 放锁
    // 时间戳给底盘的通信看门狗用：超过 500ms 没收指令就刹车
    taskENTER_CRITICAL(&cmd_spinlock);
    global_cmd.vx = msg->linear.x;
    global_cmd.vy = msg->linear.y;
    global_cmd.w  = msg->angular.z;
    global_cmd.last_update_time_us = micros();
    taskEXIT_CRITICAL(&cmd_spinlock);
}

// =====================================================================
// 订阅回调：/leader_follow → 开关跟随模式
// =====================================================================
void follow_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    leader_follow_enabled = msg->data;
}

// =====================================================================
// 定时器回调（20Hz = 每 50ms 一次）
// 负责发布 /odom、/imu、/scan（有数据就发）
// =====================================================================
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;

    // 获取 ROS 绝对时间（从 Agent 同步过来）
    int64_t stamp = rmw_uros_epoch_millis();

    // ---- 1. 从全局黑板抄录状态（Core 1 → Core 0，需要抢锁）----
    float cx = 0, cy = 0, cyaw = 0, cvx = 0, cvy = 0, cw = 0;
    float igx = 0, igy = 0, igz = 0, iax = 0, iay = 0, iaz = 0;
    float iqw = 0, iqx = 0, iqy = 0, iqz = 0;
    taskENTER_CRITICAL(&state_spinlock);
    cx   = global_state.odom_x;
    cy   = global_state.odom_y;
    cyaw = global_state.odom_yaw;
    cvx  = global_state.linear_x_speed;
    cvy  = global_state.linear_y_speed;
    cw   = global_state.angular_speed;
    igx  = global_state.imu_gyro_x; igy = global_state.imu_gyro_y; igz = global_state.imu_gyro_z;
    iax  = global_state.imu_acc_x;  iay = global_state.imu_acc_y;  iaz = global_state.imu_acc_z;
    iqw  = global_state.imu_q_w;    iqx = global_state.imu_q_x;    iqy = global_state.imu_q_y;    iqz = global_state.imu_q_z;
    taskEXIT_CRITICAL(&state_spinlock);

    // ---- 2. 发布 /odom（里程计）----
    // header.stamp 是 ROS 时间戳，pub/sub 用它对齐不同传感器的数据
    odom_msg.header.stamp.sec     = (int32_t)(stamp / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((stamp % 1000) * 1e6);
    odom_msg.pose.pose.position.x = cx;
    odom_msg.pose.pose.position.y = cy;
    // 偏航角 → 四元数（简单 2D 情况：只有绕 Z 轴旋转）
    odom_msg.pose.pose.orientation.z = sin(cyaw / 2.0);
    odom_msg.pose.pose.orientation.w = cos(cyaw / 2.0);
    odom_msg.twist.twist.linear.x  = cvx;
    odom_msg.twist.twist.linear.y  = cvy;
    odom_msg.twist.twist.angular.z = cw;
    rcl_publish(&odom_publisher, &odom_msg, NULL);

    // ---- 3. 发布 /imu（IMU 传感器数据）----
    // covariance[0] = 0.01 表示"这个数据可信度 99%"
    // robot_localization EKF 会根据协方差矩阵决定信任 IMU 还是里程计
    imu_msg.header.stamp.sec     = (int32_t)(stamp / 1000);
    imu_msg.header.stamp.nanosec = (uint32_t)((stamp % 1000) * 1e6);
    imu_msg.orientation.w = iqw; imu_msg.orientation.x = iqx;
    imu_msg.orientation.y = iqy; imu_msg.orientation.z = iqz;
    imu_msg.orientation_covariance[0] = 0.01;
    imu_msg.angular_velocity.x = igx;
    imu_msg.angular_velocity.y = igy;
    imu_msg.angular_velocity.z = igz;
    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.linear_acceleration.x = iax;
    imu_msg.linear_acceleration.y = iay;
    imu_msg.linear_acceleration.z = iaz;
    imu_msg.linear_acceleration_covariance[0] = 0.1;
    rcl_publish(&imu_publisher, &imu_msg, NULL);

    // ---- 4. 发布 /scan（激光雷达，有新帧就发）----
    // 注意：取数据时用 scan_spinlock，但只做数组拷贝（~10μs），
    // 不会像之前的 memcpy 1440 字节那样触发看门狗
    bool has_scan = false;
    taskENTER_CRITICAL(&scan_spinlock);
    if (global_scan.scan_ready) {
        laser_msg.header.stamp.sec     = (int32_t)(stamp / 1000);
        laser_msg.header.stamp.nanosec = (uint32_t)((stamp % 1000) * 1e6);
        // LaserScan 消息的标准字段
        laser_msg.angle_min       = 0.0f;
        laser_msg.angle_max       = 2.0f * PI;
        laser_msg.angle_increment = (2.0f * PI) / LIDAR_SCAN_POINTS;  // 2° = 0.0349 rad
        laser_msg.range_min       = 0.1f;   // 最小有效距离 (m)
        laser_msg.range_max       = 1.5f;   // 最大有效距离 (m)
        laser_msg.ranges.size     = LIDAR_SCAN_POINTS;
        // 拷贝 180 个 float (720 字节) — 240MHz 下只需 ~10 微秒
        for (int i = 0; i < LIDAR_SCAN_POINTS; i++) {
            laser_msg.ranges.data[i] = global_scan.ranges[i];
        }
        laser_msg.intensities.size = 0;
        global_scan.scan_ready = false;  // 取走了，等下一帧
        has_scan = true;
    }
    taskEXIT_CRITICAL(&scan_spinlock);
    if (has_scan) rcl_publish(&laser_publisher, &laser_msg, NULL);
}

// =====================================================================
// FreeRTOS 网络专职任务（Core 0，优先级 2）
// =====================================================================
void Task_MicroROSLoop(void *pvParameters) {

    // 1. 加载 WiFi 凭据：优先从 NVS 加载保存的，没有就用默认值
    if (!WebConfig_LoadWiFi(g_wifi_ssid, g_wifi_pass)) {
        strcpy(g_wifi_ssid, "iPhone");
        strcpy(g_wifi_pass, "88888888");
    }

    // 写入黑板给 OLED 显示
    taskENTER_CRITICAL(&state_spinlock);
    strncpy((char*)global_state.wifi_ssid, g_wifi_ssid, 32);
    strncpy((char*)global_state.wifi_pswd, g_wifi_pass, 32);
    global_state.wifi_status = 3;  // 状态：等待连接
    taskEXIT_CRITICAL(&state_spinlock);

    // 2. 连接 WiFi（15 秒超时）
    Serial.printf("Connecting to WiFi: %s...\n", g_wifi_ssid);
    WiFi.begin(g_wifi_ssid, g_wifi_pass);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        Serial.print(".");
        if (millis() - t0 > 15000) break;
    }

    // 3. 连接失败 → 启动 Web 配网（手机连 FishBot-Config 热点配置）
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi failed. Starting Web Config AP...");
        taskENTER_CRITICAL(&state_spinlock);
        global_state.wifi_status = 1;  // 状态：找不到 WiFi
        taskEXIT_CRITICAL(&state_spinlock);
        if (WebConfig_Start()) {
            // 用户在网页上保存了新凭据 → 重启生效
            Serial.println("Configured. Rebooting...");
            delay(500);
            ESP.restart();
        }
        // 配网超时 → 重试默认凭据
        Serial.println("Retrying defaults...");
        WiFi.begin(g_wifi_ssid, g_wifi_pass);
        while (WiFi.status() != WL_CONNECTED) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            Serial.print(".");
        }
    }

    Serial.printf("\nWiFi Connected! IP: %s\n", WiFi.localIP().toString().c_str());
    taskENTER_CRITICAL(&state_spinlock);
    global_state.wifi_status = 5;  // 状态：已获取 IP
    strncpy((char*)global_state.wifi_ip, WiFi.localIP().toString().c_str(), 16);
    taskEXIT_CRITICAL(&state_spinlock);

    // 4. 设置 Micro-ROS 传输层（UDP over WiFi）
    set_microros_wifi_transports(g_wifi_ssid, g_wifi_pass, agent_ip, agent_port, "tjk_ptw");
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    allocator = rcl_get_default_allocator();

    // 5. Ping 电脑上的 Micro-ROS Agent，直到连上为止
    Serial.println("Waiting for Micro-ROS Agent...");
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        taskENTER_CRITICAL(&state_spinlock);
        global_state.wifi_status = 4;  // 状态：Ping 失败
        taskEXIT_CRITICAL(&state_spinlock);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    Serial.println("Micro-ROS Agent Connected!");

    taskENTER_CRITICAL(&state_spinlock);
    global_state.wifi_status = 0;  // 状态：一切正常
    taskEXIT_CRITICAL(&state_spinlock);

    // 6. 初始化 ROS 节点
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "yuyu_robot_base", "", &support);

    // 7. 初始化发布者（话题名决定上位机收到的话题路径）
    rclc_publisher_init_default(&odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
    rclc_publisher_init_default(&imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu");
    rclc_publisher_init_default(&laser_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan");

    // LaserScan 的 ranges 是动态数组，需要预分配一块 buffer
    static float laser_buf[LIDAR_SCAN_POINTS] = {0};
    laser_msg.ranges.data = laser_buf;
    laser_msg.ranges.capacity = LIDAR_SCAN_POINTS;
    laser_msg.ranges.size = 0;

    // 8. 初始化订阅者
    rclc_subscription_init_default(&twist_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
    rclc_subscription_init_default(&follow_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "leader_follow");

    // 9. 50ms 定时器 (20Hz)
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);

    // 10. 初始化 Executor（3 handles = 2 订阅 + 1 定时器）
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(
        &executor, &twist_subscriber, &twist_msg, &twist_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(
        &executor, &follow_subscriber, &follow_msg, &follow_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    // 11. 同步 ROS 时间（获取绝对时间戳给 Odom 用）
    rmw_uros_sync_session(1000);

    // 12. 进入事件循环：等待话题消息 / 定时器触发
    // spin_some 不阻塞，处理挂起的消息后立刻返回
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
