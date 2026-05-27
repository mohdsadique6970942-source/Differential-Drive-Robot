#include <Arduino.h>

// ── Calibrated encoder constants ──────────────────────────────────────────────
#define MM_PER_TICK_A   0.049773f
#define MM_PER_TICK_B   0.045400f
#define WHEEL_CIRCUM_MM 138.230f

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
#define CRUISE_PWM 80

// ── Encoder pins ──────────────────────────────────────────────────────────────
#define ENC_A1  34
#define ENC_A2  35
#define ENC_B1  39
#define ENC_B2  36

// ── Encoder state ─────────────────────────────────────────────────────────────
volatile long countA = 0;
volatile long countB = 0;
volatile int  lastA  = 0;
volatile int  lastB  = 0;

// ── Speed tracking ────────────────────────────────────────────────────────────
unsigned long speedPrevTime = 0;
long          speedPrevA    = 0;
long          speedPrevB    = 0;

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

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);

  pinMode(ENC_A1, INPUT); pinMode(ENC_A2, INPUT);
  pinMode(ENC_B1, INPUT); pinMode(ENC_B2, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B2), encoderB_ISR, CHANGE);

  speedPrevTime = millis();

  motorsForward(CRUISE_PWM);

  Serial.println("\n EncA(mm)  EncB(mm)  Bot_Dist(mm)  SpdA(mm/s)  SpdB(mm/s)  Bot_Spd(mm/s)");
  Serial.println("-----------------------------------------------------------------------------");
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  static unsigned long lastPrint = 0;

  if (millis() - lastPrint >= 100) {
    lastPrint = millis();

    // snapshot counts
    noInterrupts();
    long sA = countA;
    long sB = countB;
    interrupts();

    // ── Distance ──────────────────────────────────────────────────────────────
    float mmA = sA * MM_PER_TICK_A;
    float mmB = sB * MM_PER_TICK_B;
    float avgDist = (mmA + mmB) / 2.0f;

    // ── Speed ─────────────────────────────────────────────────────────────────
    unsigned long now = millis();
    float dt = (now - speedPrevTime) / 1000.0f;
    dt = (dt < 0.001f) ? 0.001f : dt;       // guard against zero

    float spdA = ((sA - speedPrevA) * MM_PER_TICK_A) / dt;
    float spdB = ((sB - speedPrevB) * MM_PER_TICK_B) / dt;
    float avgSpd = (spdA + spdB) / 2.0f;

    speedPrevA    = sA;
    speedPrevB    = sB;
    speedPrevTime = now;

    Serial.printf("%9.2f  %8.2f  %12.2f  %10.2f  %10.2f  %13.2f\n",
      mmA, mmB, avgDist, spdA, spdB, avgSpd
    );
  }

  delay(2);
}