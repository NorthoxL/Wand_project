import serial
import time
from vpython import *

# 1. 初始化 3D 场景
scene.title = "ESP32-C3 IMU Quaternion 3D Visualization"
scene.width = 800
scene.height = 600
scene.background = color.gray(0.2)

# 创建开发板模型和坐标轴
board = box(length=5, width=3, height=0.2, color=color.cyan)
arrow_x = arrow(color=color.red, shaftwidth=0.1)
arrow_y = arrow(color=color.green, shaftwidth=0.1)
arrow_z = arrow(color=color.blue, shaftwidth=0.1)

# 2. 配置串口 (务必确认 COM 端口号是否正确)
ser = serial.Serial('COM3', 115200, timeout=0.1)
time.sleep(2)

print("Starting Quaternion 3D visualization...")

while True:
    rate(100) # 维持 100Hz 的刷新率
    
    if ser.in_waiting:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        data = line.split(',')
        
        # 确保收到了 4 个数据 (q0, q1, q2, q3)
        if len(data) == 4:
            try:
                # 提取四元数 (通常 q0 是实部 w，q1, q2, q3 是虚部 x, y, z)
                q0 = float(data[0]) 
                q1 = float(data[1]) 
                q2 = float(data[2]) 
                q3 = float(data[3]) 
                
                # --- 数学推导：四元数转方向向量 ---
                
                # 1. 计算旋转后的 X 轴向量 (物体的前方 Axis)
                vx = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
                vy = 2.0 * (q1 * q2 + q0 * q3)
                vz = 2.0 * (q1 * q3 - q0 * q2)
                
                # 2. 计算旋转后的 Z 轴向量 (物体的上方 Up)
                ux = 2.0 * (q1 * q3 + q0 * q2)
                uy = 2.0 * (q2 * q3 - q0 * q1)
                uz = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
                
                # --- 坐标系映射 (核心踩坑点) ---
                # IMU 的物理坐标系通常是：X右, Y前, Z上
                # VPython 的渲染坐标系是：X右, Y上, Z出屏幕
                # 因此我们需要做一个轴映射：VPython 的 Y 等于 IMU 的 Z，VPython 的 Z 等于 IMU 的 -Y
                
                v_axis = vector(vx, vz, -vy)
                v_up   = vector(ux, uz, -uy) 
                
                # 直接赋予绝对姿态，永杀死锁
                board.axis = v_axis
                board.up = vector(ux, uz, -uy) # 这里可能需要根据芯片丝印实际朝向微调
                
                # 让坐标系箭头跟随板子转动
                arrow_x.axis = board.axis * 3
                arrow_y.axis = board.up * 3
                arrow_z.axis = cross(board.axis, board.up) # 叉乘求第三轴
                
            except ValueError:
                pass # 忽略传输过程中的乱码