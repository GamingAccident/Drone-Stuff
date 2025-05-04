#include <Wire.h>
#define SONAR_NUM 4

void setup() {
  Serial.begin(115200);
}

int TRIG_ECHO_PIN[SONAR_NUM] = {4,2,12,14};

void loop() {
  long duration;
  float distance;
  for (uint8_t i=0; i<SONAR_NUM; i++) {
    delay(60);
    // Ορίζεις το pin ως OUTPUT για να στείλεις το trigger
    pinMode(TRIG_ECHO_PIN[i], OUTPUT);
    digitalWrite(TRIG_ECHO_PIN[i], LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_ECHO_PIN[i], HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_ECHO_PIN[i], LOW);

    // Τώρα αλλάζεις σε INPUT για να διαβάσεις το echo
    pinMode(TRIG_ECHO_PIN[i], INPUT);
    duration = pulseIn(TRIG_ECHO_PIN[i], HIGH, 30000);  // timeout 30ms

    // Υπολογισμός απόστασης
    distance = (duration * 0.0343) / 2;

    Serial.print(distance);
    Serial.print("\t");
  }
  Serial.println();
}