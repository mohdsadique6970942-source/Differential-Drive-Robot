// Download the Library "Madgwick"

#include <Wire.h>

#define MPU_ADDR        0x68
#define ACCEL_SCALE     16384.0f
#define GYRO_SCALE      131.0f
#define CALIB_SAMPLES   3000
#define CALIB_DELAY_MS  2
#define ALPHA           0.96f
#define GYRO_DEADBAND   0.5f

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

float accX_off=0, accY_off=0, accZ_off=0;
float gyroX_off=0, gyroY_off=0, gyroZ_off=0;

float roll=0, pitch=0, yaw=0;
float accRoll, accPitch;

unsigned long prevTime;
float dt;

void initMPU();
void calibrateAll();
void readMPU();
void applyOffsets();
float wrapTo360(float angle);
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  initMPU();

  Serial.println("\nPlace sensor FLAT and STILL for calibration...");
  delay(2000);

  calibrateAll();

  prevTime = micros();
  Serial.println("\nRoll\tPitch\tYaw");
}
void loop() {
  unsigned long now = micros();
  dt = (now - prevTime) * 1e-6f;
  prevTime = now;

  if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;

  readMPU();
  applyOffsets();

  accRoll  =  atan2f(accY, accZ) * RAD_TO_DEG;
  accPitch =  atan2f(-accX, sqrtf(accY*accY + accZ*accZ)) * RAD_TO_DEG;

  roll  = ALPHA * (roll  + gyroX * dt) + (1.0f - ALPHA) * accRoll;
  pitch = ALPHA * (pitch + gyroY * dt) + (1.0f - ALPHA) * accPitch;

  float gz = gyroZ;
  if (fabsf(gz) < GYRO_DEADBAND) gz = 0.0f;
  yaw += gz * dt;

  float rollOut  = wrapTo360(roll);
  float pitchOut = wrapTo360(pitch);
  float yawOut   = wrapTo360(yaw);

  Serial.print(rollOut,  2);  Serial.print("\t");
  Serial.print(pitchOut, 2);  Serial.print("\t");
  Serial.println(yawOut, 2);

  delay(5);
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x01);
  Wire.endTransmission(true);
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x03);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(200);
}

void calibrateAll() {
  Serial.println("Calibrating... DO NOT MOVE!");

  float sAX=0, sAY=0, sAZ=0;
  float sGX=0, sGY=0, sGZ=0;

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    readMPU();
    sAX += accX;  sAY += accY;  sAZ += accZ;
    sGX += gyroX; sGY += gyroY; sGZ += gyroZ;
    delay(CALIB_DELAY_MS);
    if (i % 500 == 0) Serial.print(".");
  }
  Serial.println();

  accX_off  = sAX / CALIB_SAMPLES;
  accY_off  = sAY / CALIB_SAMPLES;
  accZ_off  = (sAZ / CALIB_SAMPLES) - 1.0f;

  gyroX_off = sGX / CALIB_SAMPLES;
  gyroY_off = sGY / CALIB_SAMPLES;
  gyroZ_off = sGZ / CALIB_SAMPLES;

  Serial.println("Calibration done!");
  Serial.printf("  AccOffsets:  X=%.4f  Y=%.4f  Z=%.4f\n", accX_off, accY_off, accZ_off);
  Serial.printf("  GyroOffsets: X=%.4f  Y=%.4f  Z=%.4f\n", gyroX_off, gyroY_off, gyroZ_off);
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  accX  = (int16_t)(Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
  accY  = (int16_t)(Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;
  accZ  = (int16_t)(Wire.read() << 8 | Wire.read()) / ACCEL_SCALE;

  Wire.read(); Wire.read();

  gyroX = (int16_t)(Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
  gyroY = (int16_t)(Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
  gyroZ = (int16_t)(Wire.read() << 8 | Wire.read()) / GYRO_SCALE;
}

void applyOffsets() {
  accX  -= accX_off;
  accY  -= accY_off;
  accZ  -= accZ_off;
  gyroX -= gyroX_off;
  gyroY -= gyroY_off;
  gyroZ -= gyroZ_off;
}

float wrapTo360(float angle) {
  angle = fmodf(angle, 360.0f);      
  if (angle < 0) angle += 360.0f;   
  return angle;
}