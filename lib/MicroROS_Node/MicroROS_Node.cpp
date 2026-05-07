#include "MicroROS_Node.h"
#include "WebConfig.h"

char g_wifi_ssid[32] = {0};
char g_wifi_pass[32] = {0};
IPAddress agent_ip(172, 20, 10, 3);
const uint32_t agent_port = 8888;

rcl_publisher_t odom_publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t laser_publisher;
rcl_subscription_t twist_subscriber;
rcl_subscription_t follow_subscriber;

nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__LaserScan laser_msg;
geometry_msgs__msg__Twist twist_msg;
std_msgs__msg__Bool follow_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

bool leader_follow_enabled = false;

void twist_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
    taskENTER_CRITICAL(&cmd_spinlock);
    global_cmd.vx = msg->linear.x;
    global_cmd.vy = msg->linear.y;
    global_cmd.w  = msg->angular.z;
    global_cmd.last_update_time_us = micros();
    taskEXIT_CRITICAL(&cmd_spinlock);
}

void follow_callback(const void *msgin) {
    leader_follow_enabled = ((const std_msgs__msg__Bool *)msgin)->data;
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    if (timer == NULL) return;
    int64_t stamp = rmw_uros_epoch_millis();

    float cx = 0, cy = 0, cyaw = 0, cvx = 0, cvy = 0, cw = 0;
    taskENTER_CRITICAL(&state_spinlock);
    cx   = global_state.odom_x;
    cy   = global_state.odom_y;
    cyaw = global_state.odom_yaw;
    cvx  = global_state.linear_x_speed;
    cvy  = global_state.linear_y_speed;
    cw   = global_state.angular_speed;
    taskEXIT_CRITICAL(&state_spinlock);

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

    // --- IMU ---
    float igx = 0, igy = 0, igz = 0, iax = 0, iay = 0, iaz = 0;
    float iqw = 0, iqx = 0, iqy = 0, iqz = 0;
    taskENTER_CRITICAL(&state_spinlock);
    igx = global_state.imu_gyro_x; igy = global_state.imu_gyro_y; igz = global_state.imu_gyro_z;
    iax = global_state.imu_acc_x;  iay = global_state.imu_acc_y;  iaz = global_state.imu_acc_z;
    iqw = global_state.imu_q_w;    iqx = global_state.imu_q_x;    iqy = global_state.imu_q_y;    iqz = global_state.imu_q_z;
    taskEXIT_CRITICAL(&state_spinlock);

    imu_msg.header.stamp.sec  = (int32_t)(stamp / 1000);
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

    // --- LaserScan (有新扫帧就发) ---
    bool has_scan = false;
    taskENTER_CRITICAL(&scan_spinlock);
    if (global_scan.scan_ready) {
        laser_msg.header.stamp.sec  = (int32_t)(stamp / 1000);
        laser_msg.header.stamp.nanosec = (uint32_t)((stamp % 1000) * 1e6);
        laser_msg.angle_min = 0.0f;
        laser_msg.angle_max = 2.0f * PI;
        laser_msg.angle_increment = (2.0f * PI) / LIDAR_SCAN_POINTS;
        laser_msg.range_min = 0.1f;
        laser_msg.range_max = 1.5f;
        laser_msg.ranges.size = LIDAR_SCAN_POINTS;
        for (int i = 0; i < LIDAR_SCAN_POINTS; i++) {
            laser_msg.ranges.data[i] = global_scan.ranges[i];
        }
        laser_msg.intensities.size = 0;
        global_scan.scan_ready = false;
        has_scan = true;
    }
    taskEXIT_CRITICAL(&scan_spinlock);
    if (has_scan) {
        rcl_publish(&laser_publisher, &laser_msg, NULL);
    }
}

void Task_MicroROSLoop(void *pvParameters) {

    if (!WebConfig_LoadWiFi(g_wifi_ssid, g_wifi_pass)) {
        strcpy(g_wifi_ssid, "iPhone");
        strcpy(g_wifi_pass, "88888888");
    }

    taskENTER_CRITICAL(&state_spinlock);
    strncpy((char*)global_state.wifi_ssid, g_wifi_ssid, 32);
    strncpy((char*)global_state.wifi_pswd, g_wifi_pass, 32);
    global_state.wifi_status = 3;
    taskEXIT_CRITICAL(&state_spinlock);

    Serial.printf("Connecting to WiFi: %s...\n", g_wifi_ssid);
    WiFi.begin(g_wifi_ssid, g_wifi_pass);
    unsigned long t0 = millis();
    while (WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
        Serial.print(".");
        if (millis() - t0 > 15000) break;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nWiFi failed. Starting Web Config AP...");
        taskENTER_CRITICAL(&state_spinlock);
        global_state.wifi_status = 1;
        taskEXIT_CRITICAL(&state_spinlock);
        if (WebConfig_Start()) {
            Serial.println("Configured. Rebooting...");
            delay(500); ESP.restart();
        }
        Serial.println("Retrying defaults...");
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

    set_microros_wifi_transports(g_wifi_ssid, g_wifi_pass, agent_ip, agent_port, "tjk_ptw");
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    allocator = rcl_get_default_allocator();

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

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "yuyu_robot_base", "", &support);

    rclc_publisher_init_default(&odom_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");

    rclc_publisher_init_default(&imu_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu");

    rclc_publisher_init_default(&laser_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, LaserScan), "scan");

    static float laser_buf[LIDAR_SCAN_POINTS] = {0};
    laser_msg.ranges.data = laser_buf;
    laser_msg.ranges.capacity = LIDAR_SCAN_POINTS;
    laser_msg.ranges.size = 0;

    rclc_subscription_init_default(&twist_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");

    rclc_subscription_init_default(&follow_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool), "leader_follow");

    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(50), timer_callback);

    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_callback, ON_NEW_DATA);
    rclc_executor_add_subscription(&executor, &follow_subscriber, &follow_msg, &follow_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer);

    rmw_uros_sync_session(1000);

    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
