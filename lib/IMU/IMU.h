#ifndef IMU_H
#define IMU_H
#include "Wire.h"
#include "MPU6050_light.h"

// ==================== 1. 定义数据结构 ====================

// 定义四元数结构体 (ROS 机器人姿态的标准表达方式)
// 作用：解决三维空间旋转时的“万向节死锁”问题
typedef struct {
    float w; // 实部
    float x; // 虚部 i
    float y; // 虚部 j
    float z; // 虚部 k
} quaternion_imu_t;

// 定义三维向量结构体
// 作用：可以用来装三轴加速度，也可以装三轴角速度，还可以装欧拉角，通用性极强
typedef struct {
    float x;
    float y;
    float z;
} vector_3d_t;

// 定义终极 IMU 数据包结构体 (完美对应 ROS 里的 sensor_msgs/Imu 消息)
typedef struct {
    quaternion_imu_t orientation;      // 绝对姿态 (四元数表达)
    vector_3d_t orientation_euler;     // 绝对姿态 (人类易读的欧拉角表达，如 Roll, Pitch, Yaw)
    vector_3d_t angular_velocity;      // 角速度 (小车当前转得多快)
    vector_3d_t linear_acceleration;   // 线加速度 (小车在三个轴向上受到的推力)
} imu_t;

// ==================== 2. 定义驱动类 ====================

class ImuDriver {
private:
    // 这是一个指针，指向外部传进来的 MPU6050 硬件对象
    // 为什么要用指针？为了节省内存，并且避免在类内部重复初始化硬件
    MPU6050 *mpu_; 
    
    // 这是一个“安全锁”标志位。
    // 如果 IMU 没插紧或者坏了，它是 false，后面的读取函数就不会执行，防止主程序卡死。
    bool imu_enable_{false};

    public:
    // 构造函数：创建这个类的实例时，必须把外部的 mpu 对象传进来绑定
    ImuDriver(MPU6050 &mpu);
    ~ImuDriver() = default; // 默认析构函数

    // 核心功能接口声明 (具体实现都在 .cpp 里)
    bool begin(int sda, int scl);
    bool isEnable();
    float getYaw();
    void update();
    
    // 注意这个 static (静态函数)！
    // 静态函数意味着它是一个纯数学工具，不依赖于具体的硬件对象，随时可以直接调用
    static void Euler2Quaternion(float roll, float pitch, float yaw, quaternion_imu_t &q);

    // 数据打包出口：把你准备好的空盒子 (imu_t) 传进来，它负责把数据装满
    void getImuDriverData(imu_t &imu);
};

#endif 