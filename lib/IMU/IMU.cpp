#include "IMU.h"

// 构造函数：把外部传来的硬件对象地址，存到自己的私有指针 mpu_ 里
ImuDriver::ImuDriver(MPU6050 &mpu) {
    mpu_ = &mpu;
}

// 初始化函数：极其严谨的启动流程
bool ImuDriver::begin(int sda, int scl) {
    Wire.begin(sda, scl); // 启动 ESP32 的 I2C 总线，绑定你给的 12 和 13 引脚
    
    byte status = mpu_->begin(); // 尝试唤醒 MPU6050 芯片
    
    // 如果返回值不是 0，说明唤醒失败（可能是接线断了，或者烧了）
    if (status != 0) {
        return imu_enable_; // 此时 imu_enable_ 是 false，直接打道回府，保护程序
    }

    // 停顿 1 秒钟，让芯片内部的电容和传感器彻底稳定下来
    delay(1000); 
    
    // 执行零偏校准。极其重要！它会读取当前静止状态下的陀螺仪数据并作为“绝对零点”
    mpu_->calcOffsets(); 
    
    // 历经九九八十一难，终于初始化成功，把安全锁打开
    imu_enable_ = true;
    return imu_enable_;
}

// 获取偏航角（Z轴旋转角度）
float ImuDriver::getYaw() {
    // 指针操作符 -> 表示调用 mpu_ 指向的那个对象的 getAngleZ 方法
    return mpu_->getAngleZ();
}

// 检查 IMU 是否存活
bool ImuDriver::isEnable() {
    return imu_enable_;
}

// 更新传感器数据（必须放在主循环 loop 里高频调用）
void ImuDriver::update() {
    // 工业级保护：只有在硬件存活时才去读取寄存器，防止 I2C 总线超时卡死主程序
    if (imu_enable_) {
        mpu_->update();
    }
}

// 【最硬核的数学：欧拉角转四元数】
// 公式推导来自于 3D 图形学和航空航天动力学
// 将空间中的三次独立旋转 (围绕X、Y、Z轴) 融合为一个高维度的四元数
void ImuDriver::Euler2Quaternion(float roll, float pitch, float yaw, quaternion_imu_t &q) {
    // 为了提高计算效率，提前算出三个角度的一半的 cos 和 sin 值
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);
    
    // 四元数乘法规则展开后的标准公式（直接套用即可，这已经是人类的最优解）
    q.w = cy * cp * cr + sy * sp * sr;
    q.x = cy * cp * sr - sy * sp * cr;
    q.y = sy * cp * sr + cy * sp * cr;
    q.z = sy * cp * cr - cy * sp * sr;
}

// 数据打包中心：把所有零散的数据全部装进 imu_t 这个大盒子里
void ImuDriver::getImuDriverData(imu_t &imu) {
    // 1. 装填角速度 (Gyroscopes)
    imu.angular_velocity.x = mpu_->getGyroX();
    imu.angular_velocity.y = mpu_->getGyroY();
    imu.angular_velocity.z = mpu_->getGyroZ();

    // 2. 装填线加速度 (Accelerometers)
    imu.linear_acceleration.x = mpu_->getAccX();
    imu.linear_acceleration.y = mpu_->getAccY();
    imu.linear_acceleration.z = mpu_->getAccZ();

    // 3. 装填欧拉角 (注意物理量纲转换！)
    // RAD_2_DEG 是一个宏定义，约等于 57.2958 (即 180 / π)
    // mpu_->getAngle() 读取出来的是“度”(Degree)
    // 机器人底层算法强制要求使用“弧度”(Radian)，所以必须除以 57.2958 进行转换
    imu.orientation_euler.x = (mpu_->getAngleX() / RAD_2_DEG);
    imu.orientation_euler.y = (mpu_->getAngleY() / RAD_2_DEG);
    imu.orientation_euler.z = (mpu_->getAngleZ() / RAD_2_DEG);

    // 4. 计算并装填四元数
    // 拿着刚刚算好的弧度制欧拉角，调用上面的数学工具函数，算出四元数并存入结构体
    Euler2Quaternion(imu.orientation_euler.x, 
                     imu.orientation_euler.y, 
                     imu.orientation_euler.z,
                     imu.orientation);
}