/**
 * @file IMU.cpp
 * @brief MPU6050 六轴 IMU 传感器驱动
 *
 * MPU6050 包含：
 *   - 三轴陀螺仪：测量角速度（°/s），积分可得姿态角
 *   - 三轴加速度计：测量加速度（m/s^2），静止时指向重力方向
 *
 * 四元数 vs 欧拉角：
 *   - 欧拉角 (Roll/Pitch/Yaw) 直觉好懂，但 ±90° 会出现万向节死锁
 *   - 四元数 (w,x,y,z) 没有死锁，可以表达任意姿态，是 ROS 的标准格式
 *   - 这里两个都算出来，四元数给 ROS，欧拉角给 OLED 显示
 */

#include "IMU.h"

// 构造函数：绑定外部传入的 MPU6050 硬件对象
ImuDriver::ImuDriver(MPU6050 &mpu) {
    mpu_ = &mpu;
}

/**
 * @brief IMU 初始化（严谨启动流程）
 *
 * 1. 启动 I2C 总线
 * 2. 唤醒 MPU6050 芯片（检查是否正常应答）
 * 3. 等待 1 秒让传感器内部电容稳定
 * 4. 执行零偏校准（把当前静止状态记录为"零点"，消除零漂）
 */
bool ImuDriver::begin(int sda, int scl) {
    Wire.begin(sda, scl);
    byte status = mpu_->begin();  // 尝试唤醒芯片

    if (status != 0) {
        // 唤醒失败（接线断了？芯片烧了？）→ 不上锁，保护后续代码
        return imu_enable_;
    }

    delay(1000);           // 等传感器稳定
    mpu_->calcOffsets();   // 零偏校准
    imu_enable_ = true;    // 初始化成功，打开"安全锁"
    return imu_enable_;
}

bool ImuDriver::isEnable() { return imu_enable_; }

float ImuDriver::getYaw() { return mpu_->getAngleZ(); }

// 更新传感器数据（必须在主循环高频调用）
void ImuDriver::update() {
    // 只有硬件存活时才读取，防止 I2C 总线超时卡死
    if (imu_enable_) mpu_->update();
}

/**
 * @brief 欧拉角 → 四元数转换
 *
 * 核心思想：把三个独立的轴旋转（Roll、Pitch、Yaw）合成一个四元数
 * 公式来自 3D 图形学和航空航天动力学的标准推导
 *
 * 四元数的几何含义：
 *   - (w, x, y, z) 代表绕轴 (x, y, z) 旋转 2*acos(w) 弧度
 *   - w = cos(θ/2)，所以 w=1 表示旋转角度为 0
 */
void ImuDriver::Euler2Quaternion(float roll, float pitch, float yaw,
                                  quaternion_imu_t &q) {
    double cr = cos(roll  * 0.5), sr = sin(roll  * 0.5);
    double cp = cos(pitch * 0.5), sp = sin(pitch * 0.5);
    double cy = cos(yaw   * 0.5), sy = sin(yaw   * 0.5);

    // 这是三个旋转四元数相乘展开后的结果（已经是人类的最优解，直接套用）
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

/**
 * @brief 数据打包中心：把所有零散数据装进 imu_t 结构体
 *
 * 注意量纲转换：
 *   - MPU6050 库的 getAngle*() 返回"度"(°)，但机器人算法需要"弧度"(rad)
 *   - RAD_2_DEG ≈ 57.3，除以它 = 度转弧度
 */
void ImuDriver::getImuDriverData(imu_t &imu) {
    // 1. 角速度（陀螺仪，单位 °/s）
    imu.angular_velocity.x = mpu_->getGyroX();
    imu.angular_velocity.y = mpu_->getGyroY();
    imu.angular_velocity.z = mpu_->getGyroZ();

    // 2. 线加速度（加速度计，单位 m/s^2）
    imu.linear_acceleration.x = mpu_->getAccX();
    imu.linear_acceleration.y = mpu_->getAccY();
    imu.linear_acceleration.z = mpu_->getAccZ();

    // 3. 欧拉角（度 → 弧度）
    imu.orientation_euler.x = mpu_->getAngleX() / RAD_2_DEG;
    imu.orientation_euler.y = mpu_->getAngleY() / RAD_2_DEG;
    imu.orientation_euler.z = mpu_->getAngleZ() / RAD_2_DEG;

    // 4. 欧拉角 → 四元数
    Euler2Quaternion(imu.orientation_euler.x,
                     imu.orientation_euler.y,
                     imu.orientation_euler.z,
                     imu.orientation);
}
