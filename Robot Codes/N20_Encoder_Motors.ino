/************ TB6612FNG PINS ************/
// Motor A
#define PWMA 26
#define AIN1 14
#define AIN2 27

// Motor B
#define PWMB 32
#define BIN1 33
#define BIN2 12

#define STBY 25

/************ ENCODER PINS ************/
// Encoder A
#define ENC_A1 34
#define ENC_A2 35

// Encoder B
#define ENC_B1 39
#define ENC_B2 36

/************ PWM ************/
#define PWM_FREQ 20000
#define PWM_RES  8   // 0–255

/************ ENCODER VAR ************/
volatile long countA = 0;
volatile long countB = 0;

volatile int dirA = 0;   // 1 = CW, -1 = CCW
volatile int dirB = 0;

volatile int lastA = 0;
volatile int lastB = 0;

/************ ENCODER ISR ************/
void IRAM_ATTR encoderA_ISR() {
  int a = digitalRead(ENC_A1);
  int b = digitalRead(ENC_A2);
  int encoded = (a << 1) | b;
  int sum = (lastA << 2) | encoded;

  if (sum == 13 || sum == 4 || sum == 2 || sum == 11) {
    countA++;
    dirA = 1;   // CW
  }
  if (sum == 14 || sum == 7 || sum == 1 || sum == 8) {
    countA--;
    dirA = -1;  // CCW
  }
  lastA = encoded;
}

void IRAM_ATTR encoderB_ISR() {
  int a = digitalRead(ENC_B1);
  int b = digitalRead(ENC_B2);
  int encoded = (a << 1) | b;
  int sum = (lastB << 2) | encoded;

  if (sum == 13 || sum == 4 || sum == 2 || sum == 11) {
    countB++;
    dirB = 1;
  }
  if (sum == 14 || sum == 7 || sum == 1 || sum == 8) {
    countB--;
    dirB = -1;
  }
  lastB = encoded;
}

/************ MOTOR CONTROL ************/
void motorsForward(int pwm) {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, LOW);
  ledcWrite(PWMA, pwm);
  ledcWrite(PWMB, pwm);
}

void motorsBackward(int pwm) {
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, HIGH);
  ledcWrite(PWMA, pwm);
  ledcWrite(PWMB, pwm);
}

void motorsStop() {
  ledcWrite(PWMA, 0);
  ledcWrite(PWMB, 0);
}

/************ SETUP ************/
void setup() {
  Serial.begin(115200);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  pinMode(ENC_A1, INPUT);
  pinMode(ENC_A2, INPUT);
  pinMode(ENC_B1, INPUT);
  pinMode(ENC_B2, INPUT);

  digitalWrite(STBY, HIGH);   // Enable driver

  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);

  attachInterrupt(digitalPinToInterrupt(ENC_A1), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_A2), encoderA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B1), encoderB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B2), encoderB_ISR, CHANGE);

  Serial.println("System Ready");
}

/************ LOOP ************/
void loop() {

  // -------- FORWARD --------
  motorsForward(150);
  Serial.println("Motors: FORWARD");
  delay(2000);

  motorsStop();
  delay(500);

  Serial.print("Motor A Count: ");
  Serial.print(countA);
  Serial.print(" Direction: ");
  Serial.print(dirA == 1 ? "CW" : "CCW");

  Serial.print(" | Motor B Count: ");
  Serial.print(countB);
  Serial.print(" Direction: ");
  Serial.println(dirB == 1 ? "CW" : "CCW");

  delay(500);

  // -------- BACKWARD --------
  motorsBackward(150);
  Serial.println("Motors: BACKWARD");
  delay(2000);

  motorsStop();
  delay(500);

  Serial.print("Motor A Count: ");
  Serial.print(countA);
  Serial.print(" Direction: ");
  Serial.print(dirA == -1 ? "CCW" : "CW");

  Serial.print(" | Motor B Count: ");
  Serial.print(countB);
  Serial.print(" Direction: ");
  Serial.println(dirB == -1 ? "CCW" : "CW");

  delay(1000);
}
