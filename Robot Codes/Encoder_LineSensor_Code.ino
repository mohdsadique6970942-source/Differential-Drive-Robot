#include <Arduino.h>

// ── Calibrated encoder constants ──────────────────────────────────────────────
#define MM_PER_TICK_A   0.049773f
#define MM_PER_TICK_B   0.045400f

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

// ── Line sensor pins ──────────────────────────────────────────────────────────
// Sensors arranged left to right: LS1 LS2 LS3 LS4 LS5
#define LS1  4
#define LS2  17
#define LS3  16
#define LS4  15
#define LS5  13

// ── Cliff detection config ────────────────────────────────────────────────────
// Line sensors read HIGH on surface, LOW over cliff (no reflection)
// Adjust if your sensors are inverted
#define CLIFF_SIGNAL    LOW
#define CLIFF_THRESHOLD 3     // how many sensors must trigger to confirm cliff

// ── Encoder state ─────────────────────────────────────────────────────────────
volatile long countA = 0;
volatile long countB = 0;
volatile int  lastA  = 0;
volatile int  lastB  = 0;

// ── Globals ───────────────────────────────────────────────────────────────────
unsigned long speedPrevTime  = 0;
long          speedPrevA     = 0;
long          speedPrevB     = 0;

bool          cliffDetected  = false;
float         cliffDistMm    = 0;
unsigned long lastPrint      = 0;

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

// ── Encoder helpers ───────────────────────────────────────────────────────────
void resetEncoders() {
  noInterrupts();
  countA = 0; countB = 0;
  interrupts();
  speedPrevA = 0;
  speedPrevB = 0;
}

float encA_mm() {
  noInterrupts(); long s = countA; interrupts();
  return s * MM_PER_TICK_A;
}

float encB_mm() {
  noInterrupts(); long s = countB; interrupts();
  return s * MM_PER_TICK_B;
}

float botDist_mm() {
  return (encA_mm() + encB_mm()) / 2.0f;
}

// ── Line sensor helpers ───────────────────────────────────────────────────────

// Returns how many sensors see a cliff
int cliffCount() {
  int count = 0;
  if (digitalRead(LS1) == CLIFF_SIGNAL) count++;
  if (digitalRead(LS2) == CLIFF_SIGNAL) count++;
  if (digitalRead(LS3) == CLIFF_SIGNAL) count++;
  if (digitalRead(LS4) == CLIFF_SIGNAL) count++;
  if (digitalRead(LS5) == CLIFF_SIGNAL) count++;
  return count;
}

// Returns which side the cliff is on based on triggered sensors
const char* cliffSide() {
  bool l  = (digitalRead(LS1) == CLIFF_SIGNAL);
  bool ml = (digitalRead(LS2) == CLIFF_SIGNAL);
  bool c  = (digitalRead(LS3) == CLIFF_SIGNAL);
  bool mr = (digitalRead(LS4) == CLIFF_SIGNAL);
  bool r  = (digitalRead(LS5) == CLIFF_SIGNAL);

  int triggered = l + ml + c + mr + r;

  if (triggered >= 4)          return "FULL";
  if (l  && ml && !mr && !r)   return "LEFT";
  if (r  && mr && !ml && !l)   return "RIGHT";
  if (c  && !l && !r)          return "CENTER";
  if (l  && !r)                return "FRONT-LEFT";
  if (r  && !l)                return "FRONT-RIGHT";
  return "PARTIAL";
}

// ── Print header ──────────────────────────────────────────────────────────────
void printHeader() {
  Serial.println();
  Serial.println(" EncA(mm)  EncB(mm)  Bot_Dist(mm)  SpdA(mm/s)  SpdB(mm/s)  Bot_Spd(mm/s)  Sensors    Cliff");
  Serial.println("----------------------------------------------------------------------------------------------");
}

// ── Setup ─────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

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
  pinMode(LS1, INPUT); pinMode(LS2, INPUT); pinMode(LS3, INPUT);
  pinMode(LS4, INPUT); pinMode(LS5, INPUT);

  resetEncoders();
  speedPrevTime = millis();

  Serial.println("========================================");
  Serial.println("  ENCODER + CLIFF DETECTION SYSTEM");
  Serial.printf ("  Cliff signal  : %s\n", CLIFF_SIGNAL == LOW ? "LOW" : "HIGH");
  Serial.printf ("  Cliff threshold: %d / 5 sensors\n", CLIFF_THRESHOLD);
  Serial.printf ("  Cruise PWM    : %d / 255\n", CRUISE_PWM);
  Serial.println("========================================");

  motorsForward(CRUISE_PWM);
  printHeader();
}

// ── Loop ──────────────────────────────────────────────────────────────────────
void loop() {
  unsigned long now = millis();

  // ── Cliff check — runs every loop for fast response ──────────────────────
  if (!cliffDetected) {
    int triggered = cliffCount();
    if (triggered >= CLIFF_THRESHOLD) {
      cliffDetected = true;
      cliffDistMm   = botDist_mm();
      motorsStop();

      Serial.printf("\n!! CLIFF DETECTED at %.2f mm — Side: %s — Sensors: %d/5\n",
                    cliffDistMm, cliffSide(), triggered);
      printHeader();
    }
  }

  // ── Print every 100 ms ───────────────────────────────────────────────────
  if (now - lastPrint >= 100) {
    lastPrint = now;

    noInterrupts();
    long sA = countA;
    long sB = countB;
    interrupts();

    float mmA = sA * MM_PER_TICK_A;
    float mmB = sB * MM_PER_TICK_B;
    float avg = (mmA + mmB) / 2.0f;

    // Speed
    float dt  = (now - speedPrevTime) / 1000.0f;
    dt = (dt < 0.001f) ? 0.001f : dt;
    float spdA   = ((sA - speedPrevA) * MM_PER_TICK_A) / dt;
    float spdB   = ((sB - speedPrevB) * MM_PER_TICK_B) / dt;
    float avgSpd = (spdA + spdB) / 2.0f;
    speedPrevA   = sA;
    speedPrevB   = sB;
    speedPrevTime = now;

    // Sensor state string  e.g. "10001"
    char sensors[6];
    sensors[0] = (digitalRead(LS1) == CLIFF_SIGNAL) ? 'C' : '-';
    sensors[1] = (digitalRead(LS2) == CLIFF_SIGNAL) ? 'C' : '-';
    sensors[2] = (digitalRead(LS3) == CLIFF_SIGNAL) ? 'C' : '-';
    sensors[3] = (digitalRead(LS4) == CLIFF_SIGNAL) ? 'C' : '-';
    sensors[4] = (digitalRead(LS5) == CLIFF_SIGNAL) ? 'C' : '-';
    sensors[5] = '\0';

    const char* cliffStatus = cliffDetected
      ? "CLIFF!"
      : (cliffCount() > 0 ? "PARTIAL" : "CLEAR");

    Serial.printf("%9.2f  %8.2f  %12.2f  %10.2f  %10.2f  %13.2f  %s  %s\n",
      mmA, mmB, avg, spdA, spdB, avgSpd, sensors, cliffStatus
    );
  }

  delay(2);
}