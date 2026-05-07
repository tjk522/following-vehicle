import serial
import re
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# ==========================================
# 请将这里的 'COM3' 改成你 ESP32 实际连接的串口号 (Mac/Linux 是 /dev/ttyUSB0 等)
SERIAL_PORT = '/dev/ttyUSB0' 
BAUD_RATE = 115200
# ==========================================

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"成功连接串口 {SERIAL_PORT}")
except Exception as e:
    print(f"串口连接失败: {e}")
    exit()

# 设置极坐标图表
fig, ax = plt.subplots(subplot_kw={'projection': 'polar'})
ax.set_theta_zero_location("N") # 0度在正上方 (车头)
ax.set_theta_direction(-1)      # 顺时针方向增加角度
ax.set_ylim(0, 1500)            # 最大视距 1500 mm
ax.set_title("ESP32-S3 LiDAR Tracking Radar", va='bottom')

# 绘制正负 30 度的 ROI 扇形参考线
ax.fill_between([math.radians(-30), math.radians(30)], 0, 1500, color='green', alpha=0.1)

# 初始化目标点 (默认不在画面内)
target_point, = ax.plot([], [], 'ro', markersize=10, label="Locked Target")
ax.legend(loc="upper right")

def update(frame):
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        # 用正则抓取你 C++ 打印的 [雷达锁定] 距离: XXX mm, 角度: YYY 度
        match = re.search(r'距离:\s*([\d.]+)\s*mm,\s*角度:\s*([\d.]+)\s*度', line)
        if match:
            dist = float(match.group(1))
            angle_deg = float(match.group(2))
            
            # Matplotlib 极坐标需要弧度
            angle_rad = math.radians(angle_deg)
            target_point.set_data([angle_rad], [dist])
            ax.set_title(f"Target Locked: {dist} mm @ {angle_deg}°", color='red')
        elif "无目标" in line:
            target_point.set_data([], []) # 隐藏红点
            ax.set_title("Scanning: No Target in ROI", color='black')
            
    return target_point,

# 启动动画循环刷新
ani = animation.FuncAnimation(fig, update, interval=50, blit=False)
plt.show()