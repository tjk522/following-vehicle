#include "MicroROS_Node.h"
#include "WebConfig.h"

// --- WiFi 与 Agent 配置 ---
char g_wifi_ssid[32] = {0};
char g_wifi_pass[32] = {0};
IPAddress agent_ip(172, 20, 10, 3);
const uint32_t agent_port = 8888;

// ======================== ROS 实体 ========================

// Publishers
rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t laser_publisher;

// Subscriber
rcl_subscription_t twist_subscriber;

// Messages
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__LaserScan laser_msg;
geometry_msgs__msg__Twist twist_msg;

// Executor & node
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// ======================== Leader-Follower 开关订阅 ========================
rcl_subscription_t follow_subscriber;
std_msgs__msg__Bool follow_msg;
bool leader_follow_enabled = false;

// =====================================================================
// 订阅回调 1: cmd_vel
// =====================================================================
void twist_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    taskENTER_CRITICAL(&cmd_spinlock);
    global_cmd.vx = msg->linear.x;
    global_cmd.vy = msg->linear.y;
    global_cmd.w  = msg->angular.z;
    global_cmd.last_update_time_us = micros();
    taskEXIT_CRITICAL(&cmd_spinlock);
}

// =====================================================================
// 订阅回调 2: Leader-Follower 开关
// =====================================================================
void follow_callback(const void *msgin) {
    const std_msgs__msg__Bool *msg = (const std_msgs__msg__Bool *)msgin;
    leader_follow_enabled = msg->data;
}

// =====================================================================
// 定时器回调: Odom (20Hz) + IMU (20Hz) + LaserScan (lazy on new scan)
// =====================================================================
void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;
    int64_t stamp = rmw_uros_epoch_millis();

    // -------- 1. 读取全局状态 --------
    float cx = 0, cy = 0, cyaw = 0, cvx = 0, cvy = 0, cw = 0;
    float imu_gx = 0, imu_gy = 0, imu_gz = 0;
    float imu_ax = 0, imu_ay = 0, imu_az = 0;
    float imu_qw = 0, imu_qx = 0, imu_qy = 0, imu_qz = 0;

    taskENTER_CRITICAL(&state_spinlock);
    cx   = global_state.odom_x;
    cy   = global_state.odom_y;
    cyaw = global_state.odom_yaw;
    cvx  = global_state.linear_x_speed;
    cvy  = global_state.linear_y_speed;
    cw   = global_state.angular_speed;
    imu_gx = global_state.imu_gyro_x;
    imu_gy = global_state.imu_gyro_y;
    imu_gz = global_state.imu_gyro_z;
    imu_ax = global_state.imu_acc_x;
    imu_ay = global_state.imu_acc_y;
    imu_az = global_state.imu_acc_z;
    imu_qw = global_state.imu_q_w;
    imu_qx = global_state.imu_q_x;
    imu_qy = global_state.imu_q_y;
    imu_qz = global_state.imu_q_z;
    taskEXIT_CRITICAL(&state_spinlock);

    // -------- 2. 发布 Odom --------
    odom_msg.header.stamp.sec  = (int32_t)(stamp / 1000);
    odom_msg.header.stamp.nanosec = (uint32_t)((stamp % 1000) * 1e6);
    odom_msg.pose.pose.position.x = cx;
    odom_msg.pose.pose.position.y = cy;
    odom_msg.pose.pose.orientation.z = sin(cyaw / 2.0);
    odom_msg.pose.pose.orientation.w = cos(cyaw / 2.0);
    odom_msg.twist.twist.linear.x  = cvx;
    odom_msg.twist.twist.linear.y  = cvy;
    odom_msg.twist.twist.angular.z = cw;
    rcl_publish(&odom_publisher, &odom_msg, NULL);

    // -------- 3. 发布 IMU --------
    imu_msg.header.stamp.sec  = (int32_t)(stamp / 1000);
    imu_msg.header.stamp.nanosec = (uint32_t)((stamp % 1000) * 1e6);
    imu_msg.orientation.w = imu_qw;
    imu_msg.orientation.x = imu_qx;
    imu_msg.orientation.y = imu_qy;
    imu_msg.orientation.z = imu_qz;
    imu_msg.orientation_covariance[0] = 0.01;  // 可信
    imu_msg.angular_velocity.x = imu_gx;
    imu_msg.angular_velocity.y = imu_gy;
    imu_msg.angular_velocity.z = imu_gz;
    imu_msg.angular_velocity_covariance[0] = 0.01;
    imu_msg.linear_acceleration.x = imu_ax;
    imu_msg.linear_acceleration.y = imu_ay;
    imu_msg.linear_acceleration.z = imu_az;
    imu_msg.linear_acceleration_covariance[0] = 0.1;
    rcl_publish(&imu_publisher, &imu_msg, NULL);

    // -------- 4. 发布 LaserScan (有新扫描帧时才发) --------
    bool has_scan = false;
    taskENTER_CRITICAL(&scan_spinlock);
    has_scan = global_scan.scan_ready;
    if (has_scan) {
        laser_msg.header.stamp.sec  = (int32_t)(stamp / 1000);
        laser_msg.header.stamp.nanosec = (uint32_t)((stamp % 1000) * 1e6);
        laser_msg.angle_min = 0.0f;
        laser_msg.angle_max = 2.0f * PI;
        laser_msg.angle_increment = (2.0f * PI) / LIDAR_SCAN_POINTS;
        laser_msg.range_min = 0.1f;
        laser_msg.range_max = 1.5f;
        laser_msg.ranges.size = LIDAR_SCAN_POINTS;
        laser_msg.ranges.capacity = LIDAR_SCAN_POINTS;
        for (int i = 0; i < LIDAR_SCAN_POINTS; i++) {
            laser_msg.ranges.data[i] = global_scan.ranges[i];
        }
        laser_msg.intensities.size = 0;
        global_scan.scan_ready = false;
    }
    taskEXIT_CRITICAL(&scan_spinlock);
    if (has_scan) {
        rcl_publish(&laser_publisher, &laser_msg, NULL);
    }
}

// =====================================================================
// FreeRTOS 网络专职任务 (Core 0)
// =====================================================================
void Task_MicroROSLoop(void *pvParameters) {

    // 1. 加载保存的 WiFi 凭据
    bool has_saved = WebConfig_LoadWiFi(g_wifi_ssid, g_wifi_pass);
    if (!has_saved) {
        strcpy(g_wifi_ssid, "iPhone");
        strcpy(g_wifi_pass, "88888888");
    }

    // 写入凭据到黑板
    taskENTER_CRITICAL(&state_spinlock);
    strncpy((char*)global_state.wifi_ssid, g_wifi_ssid, 32);
    strncpy((char*)global_state.wifi_pswd, g_wifi_pass, 32);
    global_state.wifi_status = 3;
    taskEXIT_CRITICAL(&state_spinlock);

    // 2. 尝试连接 WiFi (15秒超时)
    Serial.printf("Connecting to WiFi: %s...\n", g_wifi_ssid);
    WiFi.begin(g_wifi_ssid, g_wifi_pass);

    unsigned long connect_start = millis();
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        Serial.print(".");
        if (millis() - connect_start > 15000) break;
    }

    // 3. 连接失败 → 启动 Web 配网
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi connection failed. Starting Web Config AP...");
        taskENTER_CRITICAL(&state_spinlock);
        global_state.wifi_status = 1;
        taskEXIT_CRITICAL(&state_spinlock);

        bool configured = WebConfig_Start();
        if (configured) {
            Serial.println("WiFi configured via web. Rebooting...");
            delay(500);
            ESP.restart();
        }
        // 配网超时，重试连接默认凭据
        Serial.println("Config timeout. Retrying with defaults...");
        WiFi.begin(g_wifi_ssid, g_wifi_pass);
        while (WiFi.status() != WL_CONNECTED) {
            vTaskDelay(500 / portTICK_PERIOD_MS);
            Serial.print(".");
        }
    }

    Serial.printf("\nWiFi Connected! IP: %s\n", WiFi.localIP().toString().c_str());

    taskENTER_CRITICAL(&state_spinlock);
    global_state.wifi_status = 5;
    strncpy((char*)global_state.wifi_ip, WiFi.localIP().toString().c_str(), 16);
    taskEXIT_CRITICAL(&state_spinlock);

    // 4. Micro-ROS UDP 传输层
    set_microros_wifi_transports(g_wifi_ssid, g_wifi_pass, agent_ip, agent_port, "tjk_ptw");
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    allocator = rcl_get_default_allocator();

    // 3. Ping Agent
    Serial.println("Waiting for Micro-ROS Agent...");
    while (rmw_uros_ping_agent(100, 1) != RMW_RET_OK) {
        taskENTER_CRITICAL(&state_spinlock);
        global_state.wifi_status = 4;
        taskEXIT_CRITICAL(&state_spinlock);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
    Serial.println("Micro-ROS Agent Connected!");

    taskENTER_CRITICAL(&state_spinlock);
    global_state.wifi_status = 0;
    taskEXIT_CRITICAL(&state_spinlock);

    // 4. 初始化节点
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "yuyu_robot_base", "", &support);

    // 5. 初始化发布者
    rclc_publisher_init_default(&odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");

    rclc_publisher_init_default(&imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu");

    rclc_publisher_init_default(&laser_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan");

    // 初始化 LaserScan ranges 缓冲区
    static float laser_ranges_buf[LIDAR_SCAN_POINTS] = {0};
    laser_msg.ranges.data = laser_ranges_buf;
    laser_msg.ranges.capacity = LIDAR_SCAN_POINTS;
    laser_msg.ranges.size = 0;

    // 6. 初始化订阅者
    rclc_subscription_init_default(&twist_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

    rclc_subscription_init_default(&follow_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "leader_follow");

    // 7. 定时器 50ms (20Hz)
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);

    // 8. 初始化 Executor (2 subscriptions + 1 timer = 3 handles)
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &follow_subscriber, &follow_msg, &follow_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    // 9. 同步时间
    rmw_uros_sync_session(1000);

    // 10. 网络死循环
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
