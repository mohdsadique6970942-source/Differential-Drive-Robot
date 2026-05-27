#include <Wire.h>

#define MPU_ADDR 0x68
#define GYRO_SCALE 131.0f

float gyroZ;
float gyroZ_offset = 0;

float yaw = 0;

unsigned long prevTime;
float dt;

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x43); 
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  Wire.read(); Wire.read(); 
  Wire.read(); Wire.read(); 

  gyroZ = (int16_t)(Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
}

void calibrateYaw() {
  float sum = 0;
  int samples = 2000;

  for (int i = 0; i < samples; i++) {
    readMPU();
    sum += gyroZ;
    delay(2);
  }

  gyroZ_offset = sum / samples;
}

void updateYaw() {
  unsigned long now = micros();
  dt = (now - prevTime) * 1e-6f;
  prevTime = now;

  if (dt <= 0 || dt > 0.5) dt = 0.005;

  readMPU();

  float gz = gyroZ - gyroZ_offset;

  if (fabs(gz) < 0.5f) gz = 0;

  yaw += gz * dt;

  // ✅ Wrap to 0–360
  if (yaw >= 360.0f) yaw -= 360.0f;
  if (yaw < 0.0f)    yaw += 360.0f;
}

void setup() {
  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  // Wake up MPU
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x01);
  Wire.endTransmission(true);

  delay(2000);

  Serial.println("Calibrating... keep IMU still");
  calibrateYaw();
  Serial.println("Calibration done");

  yaw = 0;
  prevTime = micros();
}

void loop() {
  updateYaw();

  Serial.print("Yaw: ");
  Serial.println(yaw, 2);

  delay(10);
}