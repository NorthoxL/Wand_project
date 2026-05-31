#include <Wire.h>
#include <SparkFun_BMI270_Arduino_Library.h>

BMI270 imu;

#define I2C_SDA 8
#define I2C_SCL 9

// --- 四元数与滤波全局变量 ---
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // 初始四元数
float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f; // 积分误差
unsigned long lastUpdate = 0; // 时间戳

// Mahony 滤波增益参数
#define Kp 2.0f   // 比例增益：决定加速度计纠正重力方向的速度
#define Ki 0.005f // 积分增益：用于消除陀螺仪的零偏误差

// ==========================================
// 新增：魔法棒手势识别 (画圆) 全局变量
// ==========================================
float last_gx = 0.0f, last_gz = 0.0f;
float cross_sum = 0.0f;
int wave_samples = 0;
const float CIRCLE_THRESHOLD = 150.0f; // 触发画圆的角速度向量模长阈值 (可根据力度微调)
const float CROSS_SUM_THRESHOLD = 15000.0f; // 动作结算的积分阈值 (越大要求画圆越完整)

void setup() {
  Serial.begin(115200);
  
  // 3秒串口等待超时
  unsigned long start_time = millis();
  while (!Serial) { if (millis() - start_time > 3000) break; delay(10); }
  delay(500);
  Serial.println("\r\n--- 启动总线修复程序 ---");

  // 【核心自救步骤】：强行释放被引脚复用锁住的总线
  pinMode(I2C_SCL, OUTPUT);
  pinMode(I2C_SDA, INPUT_PULLUP); // 让SDA保持高电平
  
  // 产生9个时钟脉冲，迫使可能卡死的IMU释放SDA数据线
  for (int i = 0; i < 9; i++) {
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(5);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(5);
  }
  
  // 确保恢复成高电平状态，给芯片一个平稳的启动环境
  digitalWrite(I2C_SCL, HIGH); 
  delay(10);

  // 重新进入标准的 I2C 初始化
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); 

  // 如果 0x69 还是报错，可以尝试改成 0x68 试试
  if (imu.beginI2C(0x69, Wire) != BMI2_OK) {
    Serial.println("错误：BMI270 初始化失败！总线仍被占用或地址错误。");
    while (1) delay(1000);
  }
  
  Serial.println("BMI270 成功连接！开始进入手势识别与姿态解算。");
  lastUpdate = micros();
}

// ==========================================
// 核心：Mahony 6DOF 四元数更新算法
// ==========================================
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    if(Ki > 0.0f) {
      integralFBx += Ki * halfex * dt;
      integralFBy += Ki * halfey * dt;
      integralFBz += Ki * halfez * dt;
      gx += integralFBx; gy += integralFBy; gz += integralFBz;
    }
    gx += Kp * halfex; gy += Kp * halfey; gz += Kp * halfez;
  }

  gx *= (0.5f * dt); gy *= (0.5f * dt); gz *= (0.5f * dt);
  qa = q0; qb = q1; qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0f;
  lastUpdate = now;

  imu.getSensorData();

  // ---------------------------------------------------------
  // 模块 1：后台执行 Mahony 算法更新四元数
  // ---------------------------------------------------------
  float gx_rad = imu.data.gyroX * PI / 180.0f;
  float gy_rad = imu.data.gyroY * PI / 180.0f;
  float gz_rad = imu.data.gyroZ * PI / 180.0f;
  float ax = imu.data.accelX;
  float ay = imu.data.accelY;
  float az = imu.data.accelZ;

  MahonyAHRSupdateIMU(gx_rad, gy_rad, gz_rad, ax, ay, az, dt);

  // ---------------------------------------------------------
  // 模块 2：魔法棒 "画圆" 手势识别核心逻辑
  // ---------------------------------------------------------
  // 提取 X 和 Z 轴原始角速度 (dps)
  float raw_gx = imu.data.gyroX;
  float raw_gz = imu.data.gyroZ;

  // 1. 计算 X-Z 平面的合角速度大小
  float magnitude = sqrt(raw_gx * raw_gx + raw_gz * raw_gz);

  // 2. 如果角速度够大，说明正在画圆挥舞
  if (magnitude > CIRCLE_THRESHOLD) {
     
     // 计算当前向量与上一次向量的二维叉乘
     float cross = last_gx * raw_gz - last_gz * raw_gx;
     
     // 累加叉乘值（过滤手抖）
     cross_sum += cross;
     wave_samples++;
     
  } else {
     // 3. 动作结束，角速度降下来了，开始结算！
     if (wave_samples > 10) { // 确保这是一个持续的画圆动作，而不是碰了一下
        
        // 叉乘总和的正负，绝对代表了画圆的方向
        if (cross_sum > CROSS_SUM_THRESHOLD) {
            Serial.println("01"); // 识别为顺时针 (或逆时针，反了请互换 01 和 02)
        } else if (cross_sum < -CROSS_SUM_THRESHOLD) {
            Serial.println("02"); 
        }
     }
     
     // 结算完毕，重置积分器，等待下一次施法
     cross_sum = 0;
     wave_samples = 0;
  }

  // 记录本次数据，供下一次叉乘使用
  last_gx = raw_gx;
  last_gz = raw_gz;

  // ---------------------------------------------------------
  // 为了让串口只输出 01 和 02 信号，暂时屏蔽四元数的打印
  // ---------------------------------------------------------
  // Serial.print(q0, 4); Serial.print(",");
  // Serial.print(q1, 4); Serial.print(",");
  // Serial.print(q2, 4); Serial.print(",");
  // Serial.println(q3, 4);

  delay(10); 
}
