#include <Wire.h>
#include <VL53L0X.h>

// ── Motor pins ────────────────────────────────────────────────────────────────
#define PWMA 26
#define AIN1 27
#define AIN2 14

#define PWMB 32
#define BIN1 33
#define BIN2 12

#define STBY 25

#define ENC1_A 34
#define ENC1_B 35
#define ENC2_A 36
#define ENC2_B 39

// ── Wheel / encoder specs — adjust to your robot ─────────────────────────────
#define WHEEL_DIAMETER_CM   6.5f   // wheel diameter in cm
#define ENCODER_CPR         20     // counts per full wheel revolution (pulses per rev)
// Distance per count = (PI * diameter) / CPR
#define CM_PER_COUNT        ((3.14159f * WHEEL_DIAMETER_CM) / ENCODER_CPR)

// ── Drive speed ───────────────────────────────────────────────────────────────
#define BASE_SPEED  80

// ── TOF stop distance ─────────────────────────────────────────────────────────
#define STOP_DISTANCE_CM  5.0f    // stop when TOF reads this close to object

// ─────────────────────────────────────────────────────────────────────────────

VL53L0X tof;

volatile long leftEncoderCount  = 0;
volatile long rightEncoderCount = 0;

void IRAM_ATTR leftEncoderISR()  { leftEncoderCount++;  }
void IRAM_ATTR rightEncoderISR() { rightEncoderCount++; }

bool running = true;

void setMotor(int left, int right);

void setup() {
  Serial.begin(115200);

  // Motor pins
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  ledcAttach(PWMA, 20000, 8);
  ledcAttach(PWMB, 20000, 8);

  // Encoder pins
  pinMode(ENC1_A, INPUT_PULLUP);
  pinMode(ENC2_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), leftEncoderISR,  RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), rightEncoderISR, RISING);

  // TOF sensor on GPIO 21 (SDA) and 22 (SCL)
  Wire.begin(21, 22);
  Wire.setClock(400000);

  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println("TOF init failed! Check wiring.");
    while (1);
  }
  tof.startContinuous(50);   // read every 50 ms

  // Reset encoder counts
  leftEncoderCount  = 0;
  rightEncoderCount = 0;

  delay(500);   // let TOF warm up

  // Print CSV header once
  Serial.println("time_s,tof_cm,left_dist_cm,right_dist_cm,avg_dist_cm");
}

void loop() {

  if (!running) {
    setMotor(0, 0);
    return;
  }

  // ── Read TOF ──────────────────────────────────────────────────────────────
  float tofCm = tof.readRangeContinuousMillimeters() / 10.0f;
  if (tof.timeoutOccurred()) tofCm = -1;   // -1 flags a bad read

  // ── Read encoders safely ──────────────────────────────────────────────────
  noInterrupts();
  long leftCount  = leftEncoderCount;
  long rightCount = rightEncoderCount;
  interrupts();

  // ── Convert counts → cm ───────────────────────────────────────────────────
  float leftDistCm  = leftCount  * CM_PER_COUNT;
  float rightDistCm = rightCount * CM_PER_COUNT;
  float avgDistCm   = (leftDistCm + rightDistCm) / 2.0f;

  // ── Stop condition: TOF sees object within STOP_DISTANCE_CM ───────────────
  if (tofCm > 0 && tofCm <= STOP_DISTANCE_CM) {
    running = false;
    setMotor(0, 0);
  } else {
    setMotor(BASE_SPEED, BASE_SPEED);
  }

  // ── Serial output: time, tof_cm, left_dist_cm, right_dist_cm, avg_dist_cm ─
  Serial.print(millis() / 1000.0, 3);
  Serial.print(",");
  Serial.print(tofCm, 1);
  Serial.print(",");
  Serial.print(leftDistCm, 2);
  Serial.print(",");
  Serial.print(rightDistCm, 2);
  Serial.print(",");
  Serial.println(avgDistCm, 2);

  delay(50);   // match TOF update rate
}

void setMotor(int left, int right) {
  if (left >= 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
  else           { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); left  = -left;  }

  if (right >= 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  }
  else            { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); right = -right; }

  ledcWrite(PWMA, left);
  ledcWrite(PWMB, right);
}
