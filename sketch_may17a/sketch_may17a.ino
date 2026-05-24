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

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  delay(1000);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); 

  if (imu.beginI2C(0x69, Wire) != BMI2_OK) {
    Serial.println("IMU Init Failed!");
    while (1);
  }
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

  // 只有加速度计数据有效时才进行融合
  if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
    
    // 1. 加速度计数据归一化
    recipNorm = 1.0f / sqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // 2. 提取当前四元数所表示的理论重力方向
    halfvx = q1 * q3 - q0 * q2;
    halfvy = q0 * q1 + q2 * q3;
    halfvz = q0 * q0 - 0.5f + q3 * q3;

    // 3. 计算理论重力与实际加速度计测量的叉积（即误差）
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // 4. 将误差进行 PI 计算，并补偿到陀螺仪角速度中
    if(Ki > 0.0f) {
      integralFBx += Ki * halfex * dt;
      integralFBy += Ki * halfey * dt;
      integralFBz += Ki * halfez * dt;
      gx += integralFBx;
      gy += integralFBy;
      gz += integralFBz;
    }
    gx += Kp * halfex;
    gy += Kp * halfey;
    gz += Kp * halfez;
  }

  // 5. 根据补偿后的角速度，更新四元数积分
  gx *= (0.5f * dt);
  gy *= (0.5f * dt);
  gz *= (0.5f * dt);
  qa = q0; qb = q1; qc = q2;
  q0 += (-qb * gx - qc * gy - q3 * gz);
  q1 += (qa * gx + qc * gz - q3 * gy);
  q2 += (qa * gy - qb * gz + q3 * gx);
  q3 += (qa * gz + qb * gy - qc * gx);

  // 6. 四元数重新归一化
  recipNorm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastUpdate) / 1000000.0f;
  lastUpdate = now;

  imu.getSensorData();

  // 必须将陀螺仪数据从 度/秒 (dps) 转换为 弧度/秒 (rad/s) 才能代入四元数微积分
  float gx_rad = imu.data.gyroX * PI / 180.0f;
  float gy_rad = imu.data.gyroY * PI / 180.0f;
  float gz_rad = imu.data.gyroZ * PI / 180.0f;

  float ax = imu.data.accelX;
  float ay = imu.data.accelY;
  float az = imu.data.accelZ;

  // 调用 Mahony 算法更新四元数
  MahonyAHRSupdateIMU(gx_rad, gy_rad, gz_rad, ax, ay, az, dt);

  // 打印输出四元数 q0, q1, q2, q3，以逗号分隔
  Serial.print(q0, 4); Serial.print(",");
  Serial.print(q1, 4); Serial.print(",");
  Serial.print(q2, 4); Serial.print(",");
  Serial.println(q3, 4);

  delay(10); 
}
