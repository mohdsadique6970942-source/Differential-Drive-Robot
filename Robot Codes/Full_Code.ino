#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define PWMA  26
#define AIN1  14
#define AIN2  27

#define PWMB  32
#define BIN1  33
#define BIN2  12

#define STBY  25

#define ENC_A1  34
#define ENC_A2  35
#define ENC_B1  39
#define ENC_B2  36

#define PWM_FREQ  20000
#define PWM_RES   8

#define LS1  4
#define LS2  17
#define LS3  16
#define LS4  15
#define LS5  13

#define MPU_ADDR       0x68
#define ACCEL_SCALE    16384.0f
#define GYRO_SCALE     131.0f
#define CALIB_SAMPLES  3000
#define CALIB_DELAY_MS 2
#define ALPHA          0.96f
#define GYRO_DEADBAND  0.5f

volatile long countA = 0;
volatile long countB = 0;
volatile int  dirA   = 0;
volatile int  dirB   = 0;
volatile int  lastA  = 0;
volatile int  lastB  = 0;

float accX, accY, accZ;
float gyroX, gyroY, gyroZ;

float accX_off=0, accY_off=0, accZ_off=0;
float gyroX_off=0, gyroY_off=0, gyroZ_off=0;

float roll=0, pitch=0, yaw=0;
float accRoll, accPitch;

unsigned long imuPrevTime;
float dt;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
uint16_t tofDistance = 0;    

unsigned long lastTofRead    = 0;
unsigned long lastDataPrint  = 0;

#define TOF_INTERVAL    100    
#define PRINT_INTERVAL  100    

void  initMPU();
void  calibrateAll();
void  readMPU();
void  applyOffsets();
float wrapTo360(float angle);
void  motorsForward(int pwm);
void  motorsBackward(int pwm);
void  motorsStop();
void  IRAM_ATTR encoderA_ISR();
void  IRAM_ATTR encoderB_ISR();

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  pinMode(AIN1, OUTPUT);  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);

  pinMode(ENC_A1, INPUT);  pinMode(ENC_A2, INPUT);
  pinMode(ENC_B1, INPUT);  pinMode(ENC_B2, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B2), encoderB_ISR, CHANGE);

  pinMode(LS1, INPUT);  pinMode(LS2, INPUT);  pinMode(LS3, INPUT);
  pinMode(LS4, INPUT);  pinMode(LS5, INPUT);

  if (!lox.begin()) {
    Serial.println("ERR: VL53L0X not found");
    while (1);
  }

  initMPU();
  Serial.println("Place robot FLAT and STILL for IMU calibration...");
  delay(2000);
  calibrateAll();

  imuPrevTime = micros();

  Serial.println("Roll\tPitch\tYaw\tToF(mm)\tLine\tEncoders");
}

void loop() {
  unsigned long now = millis();

  {
    unsigned long nowUs = micros();
    dt = (nowUs - imuPrevTime) * 1e-6f;
    imuPrevTime = nowUs;

    if (dt <= 0.0f || dt > 0.5f) dt = 0.005f;

    readMPU();
    applyOffsets();

    accRoll  = atan2f(accY, accZ) * RAD_TO_DEG;
    accPitch = atan2f(-accX, sqrtf(accY*accY + accZ*accZ)) * RAD_TO_DEG;

    roll  = ALPHA * (roll  + gyroX * dt) + (1.0f - ALPHA) * accRoll;
    pitch = ALPHA * (pitch + gyroY * dt) + (1.0f - ALPHA) * accPitch;

    float gz = gyroZ;
    if (fabsf(gz) < GYRO_DEADBAND) gz = 0.0f;
    yaw += gz * dt;
  }

  if (now - lastTofRead >= TOF_INTERVAL) {
    lastTofRead = now;
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
      tofDistance = measure.RangeMilliMeter;
    }
  }

  if (now - lastDataPrint >= PRINT_INTERVAL) {
    lastDataPrint = now;

    float rollOut  = wrapTo360(roll);

    float pitchOut = wrapTo360(pitch);

    float yawOut   = wrapTo360(yaw);

    uint16_t tofOut = tofDistance;

    char lineStr[6];
    lineStr[0] = '0' + digitalRead(LS1);
    lineStr[1] = '0' + digitalRead(LS2);
    lineStr[2] = '0' + digitalRead(LS3);
    lineStr[3] = '0' + digitalRead(LS4);
    lineStr[4] = '0' + digitalRead(LS5);
    lineStr[5] = '\0';

    noInterrupts();
    long snapA = countA;
    long snapB = countB;
    interrupts();

    Serial.print(rollOut,  2);   Serial.print("\t");
    Serial.print(pitchOut, 2);   Serial.print("\t");
    Serial.print(yawOut,   2);   Serial.print("\t");
    Serial.print(tofOut);        Serial.print("\t");
    Serial.print(lineStr);       Serial.print("\t");
    Serial.print("A:");
    Serial.print(snapA);
    Serial.print(" B:");
    Serial.println(snapB);       
  }

  delay(2);
}

void IRAM_ATTR encoderA_ISR() {
  int a = digitalRead(ENC_A1);
  int b = digitalRead(ENC_A2);
  int encoded = (a << 1) | b;
  int sum = (lastA << 2) | encoded;
  if (sum == 13 || sum == 4 || sum == 2 || sum == 11) { countA++; dirA =  1; }
  if (sum == 14 || sum == 7 || sum == 1 || sum == 8)  { countA--; dirA = -1; }
  lastA = encoded;
}

void IRAM_ATTR encoderB_ISR() {
  int a = digitalRead(ENC_B1);
  int b = digitalRead(ENC_B2);
  int encoded = (a << 1) | b;
  int sum = (lastB << 2) | encoded;
  if (sum == 13 || sum == 4 || sum == 2 || sum == 11) { countB++; dirB =  1; }
  if (sum == 14 || sum == 7 || sum == 1 || sum == 8)  { countB--; dirB = -1; }
  lastB = encoded;
}

void motorsForward(int pwm) {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  ledcWrite(PWMA, pwm);     ledcWrite(PWMB, pwm);
}

void motorsBackward(int pwm) {
  digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
  ledcWrite(PWMA, pwm);     ledcWrite(PWMB, pwm);
}

void motorsStop() {
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}

void initMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0x01);
  Wire.endTransmission(true);
  delay(100);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(0x03);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x00);
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C); Wire.write(0x00);
  Wire.endTransmission(true);

  delay(200);
}

void calibrateAll() {
  Serial.println("Calibrating IMU... DO NOT MOVE!");
  float sAX=0, sAY=0, sAZ=0, sGX=0, sGY=0, sGZ=0;

  for (int i = 0; i < CALIB_SAMPLES; i++) {
    readMPU();
    sAX += accX; sAY += accY; sAZ += accZ;
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

  Serial.println("IMU Calibration done!");
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
  accX -= accX_off; accY -= accY_off; accZ -= accZ_off;
  gyroX -= gyroX_off; gyroY -= gyroY_off; gyroZ -= gyroZ_off;
}

float wrapTo360(float angle) {
  angle = fmodf(angle, 360.0f);
  if (angle < 0) angle += 360.0f;
  return angle;
}