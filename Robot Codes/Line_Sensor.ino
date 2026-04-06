#define LS1 4
#define LS2 17
#define LS3 16
#define LS4 15
#define LS5 13

void setup() {
  Serial.begin(115200);

  pinMode(LS1, INPUT);
  pinMode(LS2, INPUT);
  pinMode(LS3, INPUT);
  pinMode(LS4, INPUT);
  pinMode(LS5, INPUT);

  Serial.println("Line Sensor @3.3V Test");
}

void loop() {
  Serial.print(digitalRead(LS1)); Serial.print("  ");
  Serial.print(digitalRead(LS2)); Serial.print("  ");
  Serial.print(digitalRead(LS3)); Serial.print("  ");
  Serial.print(digitalRead(LS4)); Serial.print("  ");
  Serial.println(digitalRead(LS5));

  delay(200);
}