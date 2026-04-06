#include <Wire.h>
#include <Adafruit_VL53L0X.h>

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);   // SDA, SCL

  Serial.println("ESP32 VL53L0X ToF Test");

  if (!lox.begin()) {
    Serial.println("❌ Failed to detect VL53L0X");
    while (1);
  }

  Serial.println("✅ VL53L0X detected");
}

void loop() {
  VL53L0X_RangingMeasurementData_t measure;

  lox.rangingTest(&measure, false);  // false = no debug print

  if (measure.RangeStatus != 4) {  // phase failure = invalid
    Serial.print("Distance: ");
    Serial.print(measure.RangeMilliMeter);
    Serial.println(" mm");
  } else {
    Serial.println("Out of range");
  }

  delay(500);
}
