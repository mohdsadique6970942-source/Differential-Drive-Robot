#include "mpu6500.h"
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "realme GT NEO 3T";       //Write your wifi name here
const char* password = "P@ssword";           //Write your wifi Password
const char * udpAddress = "10.240.171.128"; 
const int udpPort = 4210;
WiFiUDP udp;

Adafruit_VL53L0X lox = Adafruit_VL53L0X();
VL53L0X_RangingMeasurementData_t measure;

bfs::Mpu6500 imu;

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

volatile long countA = 0;
volatile long countB = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR encoderA_ISR() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(ENC1_B) == HIGH) countA++;
  else countA--;
  portEXIT_CRITICAL_ISR(&mux);
}

void IRAM_ATTR encoderB_ISR() {
  portENTER_CRITICAL_ISR(&mux);
  if (digitalRead(ENC2_B) == HIGH) countB++;
  else countB--;
  portEXIT_CRITICAL_ISR(&mux);
}

float yaw = 0;
float roll = 0;
float pitch = 0;
float gyroZ_offset = 0;
unsigned long prevTime;
float dt;

#define PWM_FREQ 20000
#define PWM_RES 8

void motorsForward(int pwm) {
  digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
  digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
  ledcWrite(PWMA, pwm); ledcWrite(PWMB, pwm);
}

void setup() {
  Serial.begin(115200);
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\nWiFi Connected!");

  Wire.begin(21, 22);
  Wire.setClock(400000);

  imu.Config(&Wire, bfs::Mpu6500::I2C_ADDR_PRIM);
  if (!imu.Begin()) {
    Serial.println("IMU init failed");
    while(1);
  }
  imu.ConfigSrd(19);

  float sum = 0;
  for(int i=0; i<500; i++) {
    if(imu.Read()) sum += imu.gyro_z_radps();
    delay(5);
  }
  gyroZ_offset = sum / 500.0;

  lox.begin();

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  ledcAttach(PWMA, PWM_FREQ, PWM_RES);
  ledcAttach(PWMB, PWM_FREQ, PWM_RES);

  pinMode(ENC1_A, INPUT);
  pinMode(ENC1_B, INPUT);
  pinMode(ENC2_A, INPUT);
  pinMode(ENC2_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENC1_A), encoderA_ISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC2_A), encoderB_ISR, RISING);

  prevTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  if (imu.Read()) {
    float gyroZ_deg = -(imu.gyro_z_radps() - gyroZ_offset) * (180.0 / PI);
    yaw += gyroZ_deg * dt;

    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;

    float ax = imu.accel_x_mps2();
    float ay = imu.accel_y_mps2();
    float az = imu.accel_z_mps2();

    float az_fixed = -az;  
    roll  = atan2(ay, az_fixed) * 180.0 / PI;
    pitch = atan2(-ax, sqrt(ay*ay + az_fixed*az_fixed)) * 180.0 / PI;
  }

  lox.rangingTest(&measure, false);
  int tof = (measure.RangeStatus != 4) ? measure.RangeMilliMeter : -1;

  long left_ticks, right_ticks;
  portENTER_CRITICAL(&mux);
  left_ticks = countA;
  right_ticks = countB;
  portEXIT_CRITICAL(&mux);

  String payload = String(yaw, 1) + "," +
                   String(left_ticks) + "," +
                   String(right_ticks) + "," +
                   String(tof) + "," +
                   String(pitch, 2) + "," +
                   String(roll, 2);

  udp.beginPacket(udpAddress, udpPort);
  udp.print(payload);
  udp.endPacket();

  Serial.println(payload);

  motorsForward(150);

  delay(20); 
}