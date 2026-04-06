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

#define BASE_SPEED     75
#define MIN_TURN_SPEED 55   

float Kp = 3.0;
float Ki = 0.02;
float Kd = 1.5;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

float accX_off=0, accY_off=0, accZ_off=0;
float gyroX_off=0, gyroY_off=0, gyroZ_off=0;

float roll=0, pitch=0, yaw=0;
float accRoll, accPitch;

unsigned long prevTime;
float dt;

float targetYaw = 0;

float pidError    = 0;
float prevError   = 0;
float integral    = 0;
float derivative  = 0;

int state = 0;
unsigned long moveStart = 0;
int sideCount = 0;

void initMPU();
void calibrateAll();
void readMPU();
void applyOffsets();
float wrapTo180(float angle);
void updateIMU();
void setMotor(int left, int right);

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

  updateIMU();
  targetYaw = yaw;

  moveStart = millis();

  Serial.println("time,yaw,target,error,correction,state,side,leftSpeed,rightSpeed");
}

void loop() {

  updateIMU();

  pidError = wrapTo180(targetYaw - yaw);

  int leftSpeed  = 0;
  int rightSpeed = 0;

  switch(state) {

    case 0: {   

      integral += pidError * dt;
      integral  = constrain(integral, -30, 30);

      if (dt > 0.001f) {
        derivative = (pidError - prevError) / dt;
      } else {
        derivative = 0;
      }

      float correction = (Kp * pidError) + (Ki * integral) + (Kd * derivative);
      prevError = pidError;

      correction = constrain(correction, -80, 80);

      leftSpeed  = BASE_SPEED + (int)correction;
      rightSpeed = BASE_SPEED - (int)correction;

      leftSpeed  = constrain(leftSpeed,  0, 255);
      rightSpeed = constrain(rightSpeed, 0, 255);

      setMotor(leftSpeed, rightSpeed);

      if (millis() - moveStart > 2000) {
        setMotor(0, 0);
        delay(200);

        targetYaw += 90;
        targetYaw  = wrapTo180(targetYaw);

        integral   = 0;
        prevError  = 0;
        derivative = 0;

        state = 1;
      }

      break;
    }

    case 1: {  

      float turnError = wrapTo180(targetYaw - yaw);

      int turnSpeed = constrain((int)(3.0f * turnError), -100, 100);

      if      (turnSpeed > 0 && turnSpeed <  MIN_TURN_SPEED) turnSpeed =  MIN_TURN_SPEED;
      else if (turnSpeed < 0 && turnSpeed > -MIN_TURN_SPEED) turnSpeed = -MIN_TURN_SPEED;

      leftSpeed  = turnSpeed;
      rightSpeed = -turnSpeed;

      leftSpeed  = constrain(leftSpeed,  -255, 255);
      rightSpeed = constrain(rightSpeed, -255, 255);

      setMotor(leftSpeed, rightSpeed);

      if (abs(turnError) < 2) {
        setMotor(0, 0);
        delay(200);

        moveStart = millis();

        integral   = 0;
        prevError  = 0;
        derivative = 0;

        state = 0;

        sideCount++;

        if (sideCount >= 4) {
          setMotor(0, 0);
          while(1);
        }
      }

      break;
    }
  }

  float currentTime = millis() / 1000.0;
  float correction  = (Kp * pidError) + (Ki * integral) + (Kd * derivative);

  Serial.print(currentTime, 3);  Serial.print(",");
  Serial.print(yaw, 2);          Serial.print(",");
  Serial.print(targetYaw, 2);    Serial.print(",");
  Serial.print(pidError, 2);     Serial.print(",");
  Serial.print(correction, 2);   Serial.print(",");
  Serial.print(state);           Serial.print(",");
  Serial.print(sideCount);       Serial.print(",");
  Serial.print(leftSpeed);       Serial.print(",");
  Serial.println(rightSpeed);

  delay(10);
}

void setMotor(int left, int right) {

  if (left >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    left = -left;
  }

  if (right >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    right = -right;
  }

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
  yaw = wrapTo180(yaw);
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