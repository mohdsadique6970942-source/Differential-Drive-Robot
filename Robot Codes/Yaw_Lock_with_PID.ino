#include <Wire.h>

#define MPU_ADDR       0x68
#define ACCEL_SCALE    16384.0f
#define GYRO_SCALE     131.0f
#define CALIB_SAMPLES  3000
#define CALIB_DELAY_MS 2
#define ALPHA          0.96f
#define GYRO_DEADBAND  0.5f

#define PWMA 26
#define AIN1 27
#define AIN2 14

#define PWMB 32
#define BIN1 33
#define BIN2 12

#define ENC1_A 34
#define ENC1_B 35
#define ENC2_A 36
#define ENC2_B 39

#define STBY 25

#define KP            5.0f   
#define KI            0.08f  
#define KD            2.0f   
#define INTEGRAL_LIMIT 30.0f 
#define MAX_SPEED      150   
#define MIN_SPEED      55    
#define DEAD_ZONE      0.5f  

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

float accX_off=0, accY_off=0, accZ_off=0;
float gyroX_off=0, gyroY_off=0, gyroZ_off=0;

float roll=0, pitch=0, yaw=0;
float accRoll, accPitch;

unsigned long prevTime;
float dt;

float targetYaw   = 0;
float integral    = 0;
float prevError   = 0;

volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;

// ✅ FIXED ISR (direction based on motor command)
void IRAM_ATTR leftEncoderISR() {
  if (digitalRead(AIN1) == HIGH && digitalRead(AIN2) == LOW)
    leftEncoderCount++;
  else
    leftEncoderCount--;
}

void IRAM_ATTR rightEncoderISR() {
  if (digitalRead(BIN1) == HIGH && digitalRead(BIN2) == LOW)
    rightEncoderCount++;
  else
    rightEncoderCount--;
}

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

  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENC1_A), leftEncoderISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), rightEncoderISR, RISING);

  Wire.begin(21, 22);
  Wire.setClock(400000);
  initMPU();

  delay(2000);
  calibrateAll();

  leftEncoderCount  = 0;
  rightEncoderCount = 0;

  prevTime = micros();
  updateIMU();

  targetYaw = yaw;
}

void loop() {

  updateIMU();

  float error = wrapTo180(targetYaw - yaw);

  if (fabsf(error) < DEAD_ZONE) {
    setMotor(0, 0);
    integral  = 0;
    prevError = 0;
  } else {
    integral += error * dt;
    integral  = constrain(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

    float derivative = 0;
    if (dt > 0.0005f) {
      derivative = (error - prevError) / dt;
    }
    prevError = error;

    float output = (KP * error) + (KI * integral) + (KD * derivative);

    int speed = (int)fabsf(output);
    speed = constrain(speed, MIN_SPEED, MAX_SPEED);

    if (output > 0) {
      setMotor( speed, -speed);   
    } else {
      setMotor(-speed,  speed);   
    }
  }

  noInterrupts();
  long leftCount  = leftEncoderCount;
  long rightCount = rightEncoderCount;
  interrupts();

  Serial.print(millis() / 1000.0, 3);
  Serial.print(",");
  Serial.print(yaw, 2);
  Serial.print(",");
  Serial.print(leftCount);
  Serial.print(",");
  Serial.println(rightCount);

  delay(10);
}

void setMotor(int left, int right) {
  if (left >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
  else           { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); left  = -left;  }

  if (right >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
  else            { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); right = -right; }

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
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x01); Wire.endTransmission(true);
  delay(100);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x00); Wire.endTransmission(true);
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
  accX -= accX_off; accY -= accY_off; accZ -= accZ_off;
  gyroX -= gyroX_off; gyroY -= gyroY_off; gyroZ -= gyroZ_off;
}

float wrapTo180(float angle) {
  angle = fmodf(angle + 180.0f, 360.0f);
  if (angle < 0) angle += 360.0f;
  return angle - 180.0f;
}