#include <Wire.h>

#define MPU_ADDR 0x68
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE 131.0f
#define CALIB_SAMPLES 3000
#define CALIB_DELAY_MS 2
#define ALPHA 0.96f
#define GYRO_DEADBAND 0.5f

#define PWMA 26
#define AIN1 27
#define AIN2 14

#define PWMB 32
#define BIN1 33
#define BIN2 12

#define STBY 25

#define BASE_SPEED 80

#define KP 3.0f
#define KI 0.01f
#define KD 1.2f
#define INTEGRAL_LIMIT 50.0f   

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

float accX_off=0, accY_off=0, accZ_off=0;
float gyroX_off=0, gyroY_off=0, gyroZ_off=0;

float roll=0, pitch=0, yaw=0;
float accRoll, accPitch;

unsigned long prevTime;
float dt;

float targetYaw = 0;

float pidIntegral  = 0;
float prevError    = 0;

void initMPU();
void calibrateAll();
void readMPU();
void applyOffsets();
float wrapTo180(float angle);
void updateIMU();
void setMotor(int left, int right);
float computePID(float error, float dt);

void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);

  ledcAttach(PWMA, 20000, 8);
  ledcAttach(PWMB, 20000, 8);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  initMPU();

  delay(2000);
  calibrateAll();

  prevTime = micros();
}

void loop() {

  updateIMU();

  float error = wrapTo180(yaw - targetYaw);

  float correction = computePID(error, dt);

  int leftSpeed  = BASE_SPEED - correction;
  int rightSpeed = BASE_SPEED + correction;

  leftSpeed  = constrain(leftSpeed,  0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  setMotor(leftSpeed, rightSpeed);

  Serial.print("Yaw_err: "); Serial.print(error);
  Serial.print("  PID: ");   Serial.print(correction);
  Serial.print("  L: ");     Serial.print(leftSpeed);
  Serial.print("  R: ");     Serial.println(rightSpeed);

  delay(5);
}

float computePID(float error, float dt) {

  float P = KP * error;

  pidIntegral += error * dt;
  pidIntegral  = constrain(pidIntegral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  float I = KI * pidIntegral;

  float D = 0;
  if (dt > 0) {
    D = KD * (error - prevError) / dt;
  }
  prevError = error;

  return P + I + D;
}

void setMotor(int left, int right) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);

  ledcWrite(PWMA, left);
  ledcWrite(PWMB, right);
}

void updateIMU() {

  unsigned long now = micros();
  dt = (now - prevTime) * 1e-6f;
  prevTime = now;

  if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;

  readMPU();
  applyOffsets();

  accRoll  = atan2f(accY, accZ) * RAD_TO_DEG;
  accPitch = atan2f(-accX, sqrtf(accY*accY + accZ*accZ)) * RAD_TO_DEG;

  roll  = ALPHA * (roll  + gyroX * dt) + (1 - ALPHA) * accRoll;
  pitch = ALPHA * (pitch + gyroY * dt) + (1 - ALPHA) * accPitch;

  float gz = gyroZ;
  if (fabsf(gz) < GYRO_DEADBAND) gz = 0;

  yaw += gz * dt;
  yaw  = wrapTo180(yaw);
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
  float sAX=0, sAY=0, sAZ=0;
  float sGX=0, sGY=0, sGZ=0;

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    readMPU();
    sAX += accX; sAY += accY; sAZ += accZ;
    sGX += gyroX; sGY += gyroY; sGZ += gyroZ;
    delay(CALIB_DELAY_MS);
  }

  accX_off = sAX / CALIB_SAMPLES;
  accY_off = sAY / CALIB_SAMPLES;
  accZ_off = (sAZ / CALIB_SAMPLES) - 1.0f;

  gyroX_off = sGX / CALIB_SAMPLES;
  gyroY_off = sGY / CALIB_SAMPLES;
  gyroZ_off = sGZ / CALIB_SAMPLES;
}

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  accX = (int16_t)(Wire.read()<<8 | Wire.read()) / ACCEL_SCALE;
  accY = (int16_t)(Wire.read()<<8 | Wire.read()) / ACCEL_SCALE;
  accZ = (int16_t)(Wire.read()<<8 | Wire.read()) / ACCEL_SCALE;

  Wire.read(); Wire.read();

  gyroX = (int16_t)(Wire.read()<<8 | Wire.read()) / GYRO_SCALE;
  gyroY = (int16_t)(Wire.read()<<8 | Wire.read()) / GYRO_SCALE;
  gyroZ = (int16_t)(Wire.read()<<8 | Wire.read()) / GYRO_SCALE;
}

void applyOffsets() {
  accX -= accX_off;
  accY -= accY_off;
  accZ -= accZ_off;

  gyroX -= gyroX_off;
  gyroY -= gyroY_off;
  gyroZ -= gyroZ_off;
}

float wrapTo180(float angle) {
  angle = fmodf(angle + 180.0f, 360.0f);
  if (angle < 0) angle += 360.0f;
  return angle - 180.0f;
}