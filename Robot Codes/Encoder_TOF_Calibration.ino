#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ── Robot physical constants ──────────────────────────────────────────────────
#define WHEEL_DIAMETER_MM   44.0f
#define WHEEL_BASE_MM       134.0f
#define WHEEL_CIRCUM_MM     138.230f

// ── Calibrated encoder constants ──────────────────────────────────────────────
#define MM_PER_TICK_A       0.049773f
#define MM_PER_TICK_B       0.045400f
#define TICKS_PER_MM_A      20.0911f
#define TICKS_PER_MM_B      19.8411f

// ── Safety threshold ──────────────────────────────────────────────────────────
#define TOF_STOP_MM         300
#define TOF_INTERVAL_MS     50
#define PRINT_INTERVAL_MS   100

// ── Motor pins ────────────────────────────────────────────────────────────────
#define PWMA   26
#define AIN1   27
#define AIN2   14
#define PWMB   32
#define BIN1   33
#define BIN2   12
#define STBY   25

#define PWM_FREQ   20000
#define PWM_RES    8

// ── Encoder pins ──────────────────────────────────────────────────────────────
#define ENC_A1  34
#define ENC_A2  35
#define ENC_B1  39
#define ENC_B2  36

// ── Cruise PWM (180 - 20% = 144) ─────────────────────────────────────────────
#define CRUISE_PWM  80

// ── Structs ───────────────────────────────────────────────────────────────────
struct Odometry {
  float x         = 0;
  float y         = 0;
  float heading   = 0;
  float distTotal = 0;
};

struct WheelSpeed {
  float rpmA;
  float rpmB;
  float mmpsA;
  float mmpsB;
};

// ── Encoder state ─────────────────────────────────────────────────────────────
volatile long countA = 0;
volatile long countB = 0;
volatile int  lastA  = 0;
volatile int  lastB  = 0;

// ── Globals ───────────────────────────────────────────────────────────────────
Odometry      odom;
long          prevA         = 0;
long          prevB         = 0;
unsigned long speedPrevTime = 0;
long          speedPrevA    = 0;
long          speedPrevB    = 0;

Adafruit_VL53L0X lox;
uint16_t      tofDistance   = 9999;
uint16_t      tofStart      = 0;
unsigned long lastTofRead   = 0;
unsigned long lastPrint     = 0;

bool          robotRunning  = false;
bool          stoppedByTof  = false;

// ── ISRs ──────────────────────────────────────────────────────────────────────
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

// ── Motor control ─────────────────────────────────────────────────────────────
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

// ── Helpers ───────────────────────────────────────────────────────────────────
void resetAll() {
  noInterrupts();
  countA = 0; countB = 0;
  interrupts();
  prevA = 0; prevB = 0;
  odom  = {0, 0, 0, 0};
}

void updateOdometry() {
  noInterrupts();
  long snapA = countA;
  long snapB = countB;
  interrupts();

  long dA = snapA - prevA;
  long dB = snapB - prevB;
  prevA = snapA;
  prevB = snapB;

  float dL = dA * MM_PER_TICK_A;
  float dR = dB * MM_PER_TICK_B;

  float dCentre = (dL + dR) / 2.0f;
  float dTheta  = (dR - dL) / WHEEL_BASE_MM;

  float hRad = odom.heading * DEG_TO_RAD;
  odom.x        += dCentre * cosf(hRad + dTheta / 2.0f);
  odom.y        += dCentre * sinf(hRad + dTheta / 2.0f);
  odom.heading  += dTheta * RAD_TO_DEG;
  odom.distTotal += fabsf(dCentre);

  if (odom.heading >  180.0f) odom.heading -= 360.0f;
  if (odom.heading < -180.0f) odom.heading += 360.0f;
}

WheelSpeed getWheelSpeed() {
  unsigned long now = millis();
  float dt = (now - speedPrevTime) / 1000.0f;
  if (dt < 0.01f) return {0, 0, 0, 0};

  noInterrupts();
  long sA = countA;
  long sB = countB;
  interrupts();

  float mmpsA = ((sA - speedPrevA) * MM_PER_TICK_A) / dt;
  float mmpsB = ((sB - speedPrevB) * MM_PER_TICK_B) / dt;

  speedPrevA    = sA;
  speedPrevB    = sB;
  speedPrevTime = now;

  return { (mmpsA / WHEEL_CIRCUM_MM) * 60.0f,
           (mmpsB / WHEEL_CIRCUM_MM) * 60.0f,
           mmpsA, mmpsB };
}

void printHeader() {
  Serial.println();
  Serial.println(" ToF_Dist(mm)  EncA_Dist(mm)  EncB_Dist(mm)  Avg_Enc_Dist(mm)");
  Serial.println("-----------------------------------------------------------------");
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);
  motorsStop();

  pinMode(ENC_A1, INPUT); pinMode(ENC_A2, INPUT);
  pinMode(ENC_B1, INPUT); pinMode(ENC_B2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B2), encoderB_ISR, CHANGE);

  if (!lox.begin()) {
    Serial.println("ERR: VL53L0X not found!");
    while (1);
  }

  delay(100);
  VL53L0X_RangingMeasurementData_t m;
  lox.rangingTest(&m, false);
  tofStart    = (m.RangeStatus != 4) ? m.RangeMilliMeter : 9999;
  tofDistance = tofStart;

  resetAll();
  speedPrevTime = millis();

  robotRunning = true;
  motorsForward(CRUISE_PWM);

  Serial.println("========================================");
  Serial.println("  ENCODER + TOF DRIVE SYSTEM");
  Serial.printf ("  Stop threshold : %d mm (%.0f cm)\n", TOF_STOP_MM, TOF_STOP_MM / 10.0f);
  Serial.printf ("  Cruise PWM     : %d / 255\n", CRUISE_PWM);
  Serial.printf ("  ToF at start   : %d mm\n", tofStart);
  Serial.println("========================================");
  printHeader();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── ToF read ────────────────────────────────────────────────────────────────
  if (now - lastTofRead >= TOF_INTERVAL_MS) {
    lastTofRead = now;

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);
    if (measure.RangeStatus != 4) {
      tofDistance = measure.RangeMilliMeter;
    }

    if (robotRunning && tofDistance <= TOF_STOP_MM) {
      robotRunning = false;
      stoppedByTof = true;
      motorsStop();

      noInterrupts();
      long sA = countA;
      long sB = countB;
      interrupts();
      float mmA  = sA * MM_PER_TICK_A;
      float mmB  = sB * MM_PER_TICK_B;
      float avg  = (mmA + mmB) / 2.0f;

      Serial.printf("\n!! OBSTACLE at %d mm — STOPPED\n", tofDistance);
      Serial.printf("   EncA: %.2f mm  EncB: %.2f mm  Avg: %.2f mm\n", mmA, mmB, avg);
      printHeader();
    }
  }

  // ── Odometry ─────────────────────────────────────────────────────────────────
  updateOdometry();

  // ── Print ─────────────────────────────────────────────────────────────────────
  if (now - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = now;

    getWheelSpeed();   // keep speed internals ticking even though not printed

    noInterrupts();
    long sA = countA;
    long sB = countB;
    interrupts();

    float mmA = sA * MM_PER_TICK_A;
    float mmB = sB * MM_PER_TICK_B;
    float avg = (mmA + mmB) / 2.0f;

    Serial.printf("%13d  %13.2f  %13.2f  %16.2f\n",
      tofDistance,
      mmA,
      mmB,
      avg
    );
  }

  delay(2);
}