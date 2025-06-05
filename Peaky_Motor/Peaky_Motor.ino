volatile int rotations = 0;
unsigned long prevStartPoint = 0;

void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(4), wow, FALLING);
}

void loop() {
  if (millis() - prevStartPoint >= 1000) {
    prevStartPoint = millis();
    Serial.print(millis()); Serial.print("\t"); Serial.println(rotations);
    rotations = 0;
  }
}

void wow() {
  rotations++;
}