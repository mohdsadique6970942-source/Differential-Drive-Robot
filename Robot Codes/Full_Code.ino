#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ── Motor pins ────────────────────────────────────────────────────────────────
#define PWMA  26
#define AIN1  27
#define AIN2  14
#define PWMB  32
#define BIN1  33
#define BIN2  12
#define STBY  25

#define PWM_FREQ   20000
#define PWM_RES    8
#define CRUISE_PWM 144

// ── Encoder pins ──────────────────────────────────────────────────────────────
#define ENC_A1  34
#define ENC_A2  35
#define ENC_B1  39
#define ENC_B2  36

// ── Line sensor pins ──────────────────────────────────────────────────────────
#define LS1  4
#define LS2  17
#define LS3  16
#define LS4  15
#define LS5  13

// ── IMU ───────────────────────────────────────────────────────────────────────
#define MPU_ADDR       0x68
#define ACCEL_SCALE    16384.0f
#define GYRO_SCALE     131.0f
#define CALIB_SAMPLES  3000
#define CALIB_DELAY_MS 2
#define ALPHA          0.96f
#define GYRO_DEADBAND  0.5f

// ── Encoder calibration ───────────────────────────────────────────────────────
#define MM_PER_TICK_A   0.049773f
#define MM_PER_TICK_B   0.050400f
#define WHEEL_CIRCUM_MM 138.230f

// ── Safety thresholds ─────────────────────────────────────────────────────────
#define TOF_STOP_MM        300    // stop if object within 30 cm
#define CLIFF_SIGNAL       LOW    // sensor reads LOW over cliff
#define CLIFF_THRESHOLD    3      // min sensors to confirm cliff
#define TOF_INTERVAL_MS    50
#define PRINT_INTERVAL_MS  100

// ── Encoder state ─────────────────────────────────────────────────────────────
volatile long countA = 0;
volatile long countB = 0;
volatile int  lastA  = 0;
volatile int  lastB  = 0;

// ── IMU state ─────────────────────────────────────────────────────────────────
float accX, accY, accZ;
float gyroX, gyroY, gyroZ;
float accX_off=0, accY_off=0, accZ_off=0;
float gyroX_off=0, gyroY_off=0, gyroZ_off=0;
float roll=0, pitch=0, yaw=0;
unsigned long imuPrevTime;
float imuDt;

// ── ToF ───────────────────────────────────────────────────────────────────────
Adafruit_VL53L0X lox;
uint16_t tofDistance  = 9999;
uint16_t tofStart     = 0;
unsigned long lastTofRead = 0;

// ── Speed tracking ────────────────────────────────────────────────────────────
unsigned long speedPrevTime = 0;
long          speedPrevA    = 0;
long          speedPrevB    = 0;

// ── Robot state ───────────────────────────────────────────────────────────────
bool robotRunning  = false;
bool stoppedByTof  = false;
bool stoppedByCliff= false;
float cliffDistMm  = 0;

unsigned long lastPrint = 0;

// ─────────────────────────────────────────────────────────────────────────────
//  ISRs
// ─────────────────────────────────────────────────────────────────────────────
void IRAM_ATTR encoderA_ISR() {
  int a = digitalRead(ENC_A1);
  int b = digitalRead(ENC_A2);
  int encoded = (a << 1) | b;
  int sum = (lastA << 2) | encoded;
  if (sum == 13 || sum == 4 || sum == 2 || sum == 11) countA++;
  if (sum == 14 || sum == 7 || sum == 1 || sum == 8)  countA--;
  lastA = encoded;
}

void IRAM_ATTR encoderB_ISR() {
  int a = digitalRead(ENC_B1);
  int b = digitalRead(ENC_B2);
  int encoded = (a << 1) | b;
  int sum = (lastB << 2) | encoded;
  if (sum == 13 || sum == 4 || sum == 2 || sum == 11) countB++;
  if (sum == 14 || sum == 7 || sum == 1 || sum == 8)  countB--;
  lastB = encoded;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Motor control
// ─────────────────────────────────────────────────────────────────────────────
void motorsForward(int pwm) {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  ledcWrite(PWMA, pwm);
  ledcWrite(PWMB, pwm);
}

void motorsStop() {
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}

// ─────────────────────────────────────────────────────────────────────────────
//  Encoder helpers
// ─────────────────────────────────────────────────────────────────────────────
void resetEncoders() {
  noInterrupts();
  countA = 0; countB = 0;
  interrupts();
  speedPrevA = 0; speedPrevB = 0;
}

float encA_mm() { noInterrupts(); long s=countA; interrupts(); return s*MM_PER_TICK_A; }
float encB_mm() { noInterrupts(); long s=countB; interrupts(); return s*MM_PER_TICK_B; }
float botDist_mm() { return (encA_mm() + encB_mm()) / 2.0f; }

// ─────────────────────────────────────────────────────────────────────────────
//  Line sensor helpers
// ─────────────────────────────────────────────────────────────────────────────
int cliffCount() {
  return (digitalRead(LS1)==CLIFF_SIGNAL) +
         (digitalRead(LS2)==CLIFF_SIGNAL) +
         (digitalRead(LS3)==CLIFF_SIGNAL) +
         (digitalRead(LS4)==CLIFF_SIGNAL) +
         (digitalRead(LS5)==CLIFF_SIGNAL);
}

const char* cliffSide() {
  bool l  = (digitalRead(LS1)==CLIFF_SIGNAL);
  bool ml = (digitalRead(LS2)==CLIFF_SIGNAL);
  bool c  = (digitalRead(LS3)==CLIFF_SIGNAL);
  bool mr = (digitalRead(LS4)==CLIFF_SIGNAL);
  bool r  = (digitalRead(LS5)==CLIFF_SIGNAL);
  int  n  = l+ml+c+mr+r;
  if (n >= 4)          return "FULL";
  if (l  && ml && !r)  return "LEFT";
  if (r  && mr && !l)  return "RIGHT";
  if (c  && !l && !r)  return "CENTER";
  if (l  && !r)        return "FRONT-LEFT";
  if (r  && !l)        return "FRONT-RIGHT";
  return "PARTIAL";
}

// ─────────────────────────────────────────────────────────────────────────────
//  IMU
// ─────────────────────────────────────────────────────────────────────────────
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

void readMPU() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)true);

  accX  = (int16_t)(Wire.read()<<8 | Wire.read()) / ACCEL_SCALE;
  accY  = (int16_t)(Wire.read()<<8 | Wire.read()) / ACCEL_SCALE;
  accZ  = (int16_t)(Wire.read()<<8 | Wire.read()) / ACCEL_SCALE;
  Wire.read(); Wire.read();
  gyroX = (int16_t)(Wire.read()<<8 | Wire.read()) / GYRO_SCALE;
  gyroY = (int16_t)(Wire.read()<<8 | Wire.read()) / GYRO_SCALE;
  gyroZ = (int16_t)(Wire.read()<<8 | Wire.read()) / GYRO_SCALE;
}

void applyOffsets() {
  accX -= accX_off; accY -= accY_off; accZ -= accZ_off;
  gyroX -= gyroX_off; gyroY -= gyroY_off; gyroZ -= gyroZ_off;
}

void calibrateIMU() {
  Serial.println("Calibrating IMU... DO NOT MOVE!");
  float sAX=0,sAY=0,sAZ=0,sGX=0,sGY=0,sGZ=0;
  for (int i=0; i<CALIB_SAMPLES; i++) {
    readMPU();
    sAX+=accX; sAY+=accY; sAZ+=accZ;
    sGX+=gyroX; sGY+=gyroY; sGZ+=gyroZ;
    delay(CALIB_DELAY_MS);
    if (i%500==0) Serial.print(".");
  }
  Serial.println();
  accX_off  = sAX/CALIB_SAMPLES;
  accY_off  = sAY/CALIB_SAMPLES;
  accZ_off  = (sAZ/CALIB_SAMPLES)-1.0f;
  gyroX_off = sGX/CALIB_SAMPLES;
  gyroY_off = sGY/CALIB_SAMPLES;
  gyroZ_off = sGZ/CALIB_SAMPLES;
  Serial.println("IMU calibration done!");
  Serial.printf("  Acc  offsets: X=%.4f Y=%.4f Z=%.4f\n", accX_off,  accY_off,  accZ_off);
  Serial.printf("  Gyro offsets: X=%.4f Y=%.4f Z=%.4f\n", gyroX_off, gyroY_off, gyroZ_off);
}

void updateIMU() {
  unsigned long nowUs = micros();
  imuDt = (nowUs - imuPrevTime) * 1e-6f;
  imuPrevTime = nowUs;
  if (imuDt <= 0.0f || imuDt > 0.5f) imuDt = 0.005f;

  readMPU();
  applyOffsets();

  float accRoll  = atan2f(accY, accZ) * RAD_TO_DEG;
  float accPitch = atan2f(-accX, sqrtf(accY*accY + accZ*accZ)) * RAD_TO_DEG;

  roll  = ALPHA*(roll  + gyroX*imuDt) + (1.0f-ALPHA)*accRoll;
  pitch = ALPHA*(pitch + gyroY*imuDt) + (1.0f-ALPHA)*accPitch;

  float gz = (fabsf(gyroZ) < GYRO_DEADBAND) ? 0.0f : gyroZ;
  yaw += gz * imuDt;

  // wrap yaw to 0-360
  yaw = fmodf(yaw, 360.0f);
  if (yaw < 0) yaw += 360.0f;
}

float wrapTo360(float a) {
  a = fmodf(a, 360.0f);
  if (a < 0) a += 360.0f;
  return a;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Print header
// ─────────────────────────────────────────────────────────────────────────────
void printHeader() {
  Serial.println();
  Serial.println(" Roll(°) Pitch(°)  Yaw(°) | ToF(mm) | EncA(mm) EncB(mm) Bot(mm) | SpdA(mm/s) SpdB(mm/s) BotSpd(mm/s) | Sensors | State");
  Serial.println("-------------------------------------------------------------------------------------------------------------------------------");
}

// ─────────────────────────────────────────────────────────────────────────────
//  Setup
// ─────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  // Motors
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
  motorsStop();

  // Encoders
  pinMode(ENC_A1, INPUT); pinMode(ENC_A2, INPUT);
  pinMode(ENC_B1, INPUT); pinMode(ENC_B2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B2), encoderB_ISR, CHANGE);

  // Line sensors
  pinMode(LS1,INPUT); pinMode(LS2,INPUT); pinMode(LS3,INPUT);
  pinMode(LS4,INPUT); pinMode(LS5,INPUT);

  // ToF
  if (!lox.begin()) {
    Serial.println("ERR: VL53L0X not found!");
    while (1);
  }

  // IMU
  initMPU();
  Serial.println("Place robot FLAT and STILL for IMU calibration...");
  delay(2000);
  calibrateIMU();

  // Snapshot ToF at start
  delay(100);
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  tofStart    = (m.RangeStatus != 4) ? m.RangeMilliMeter : 9999;
  tofDistance = tofStart;

  resetEncoders();
  speedPrevTime = millis();
  imuPrevTime   = micros();

  Serial.println("========================================");
  Serial.println("  FULL SENSOR DRIVE SYSTEM");
  Serial.printf ("  ToF stop      : %d mm\n", TOF_STOP_MM);
  Serial.printf ("  Cliff trigger : %d / 5 sensors\n", CLIFF_THRESHOLD);
  Serial.printf ("  Cruise PWM    : %d / 255\n", CRUISE_PWM);
  Serial.printf ("  ToF at start  : %d mm\n", tofStart);
  Serial.println("========================================");

  robotRunning = true;
  motorsForward(CRUISE_PWM);
  printHeader();
}

// ─────────────────────────────────────────────────────────────────────────────
//  Loop
// ─────────────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── IMU — update every loop ───────────────────────────────────────────────
  updateIMU();

  // ── ToF — every 50 ms ─────────────────────────────────────────────────────
  if (now - lastTofRead >= TOF_INTERVAL_MS) {
    lastTofRead = now;
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) tofDistance = measure.RangeMilliMeter;

    if (robotRunning && tofDistance <= TOF_STOP_MM) {
      robotRunning = false;
      stoppedByTof = true;
      motorsStop();
      Serial.printf("\n!! TOF STOP — obstacle at %d mm — Bot travelled %.2f mm\n",
                    tofDistance, botDist_mm());
      printHeader();
    }
  }

  // ── Cliff — every loop for fast response ──────────────────────────────────
  if (robotRunning) {
    int cc = cliffCount();
    if (cc >= CLIFF_THRESHOLD) {
      robotRunning   = false;
      stoppedByCliff = true;
      cliffDistMm    = botDist_mm();
      motorsStop();
      Serial.printf("\n!! CLIFF STOP — side: %s — %d/5 sensors — at %.2f mm\n",
                    cliffSide(), cc, cliffDistMm);
      printHeader();
    }
  }

  // ── Print — every 100 ms ──────────────────────────────────────────────────
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;

    // Encoder distances
    noInterrupts();
    long sA = countA;
    long sB = countB;
    interrupts();
    float mmA = sA * MM_PER_TICK_A;
    float mmB = sB * MM_PER_TICK_B;
    float avg = (mmA + mmB) / 2.0f;

    // Wheel speeds
    float dt  = (now - speedPrevTime) / 1000.0f;
    dt = (dt < 0.001f) ? 0.001f : dt;
    float spdA   = ((sA - speedPrevA) * MM_PER_TICK_A) / dt;
    float spdB   = ((sB - speedPrevB) * MM_PER_TICK_B) / dt;
    float avgSpd = (spdA + spdB) / 2.0f;
    speedPrevA    = sA;
    speedPrevB    = sB;
    speedPrevTime = now;

    // Sensor bar  C=cliff  -=clear
    char sensors[6];
    sensors[0] = (digitalRead(LS1)==CLIFF_SIGNAL) ? 'C' : '-';
    sensors[1] = (digitalRead(LS2)==CLIFF_SIGNAL) ? 'C' : '-';
    sensors[2] = (digitalRead(LS3)==CLIFF_SIGNAL) ? 'C' : '-';
    sensors[3] = (digitalRead(LS4)==CLIFF_SIGNAL) ? 'C' : '-';
    sensors[4] = (digitalRead(LS5)==CLIFF_SIGNAL) ? 'C' : '-';
    sensors[5] = '\0';

    // State label
    const char* state = robotRunning   ? " MOVING " :
                        stoppedByTof   ? "TOF-STOP" :
                        stoppedByCliff ? "CLF-STOP" : "STOPPED ";

    Serial.printf("%7.2f  %7.2f  %6.2f | %7d | %8.2f %8.2f %7.2f | %10.2f %10.2f %11.2f | %s | %s\n",
      wrapTo360(roll),
      wrapTo360(pitch),
      yaw,
      tofDistance,
      mmA, mmB, avg,
      spdA, spdB, avgSpd,
      sensors,
      state
    );
  }

  delay(2);
}