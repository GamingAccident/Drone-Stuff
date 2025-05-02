#define TRIG_ECHO_PIN_1 13
#define TRIG_ECHO_PIN_2 10
#define TRIG_ECHO_PIN_3 9
#define TRIG_ECHO_PIN_4 11

void setup() {
  Serial.begin(115200);
}

void loop() {
  long duration;
  float distance;
  // Ορίζεις το pin ως OUTPUT για να στείλεις το trigger
  pinMode(TRIG_ECHO_PIN_1, OUTPUT);
  digitalWrite(TRIG_ECHO_PIN_1, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_ECHO_PIN_1, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_ECHO_PIN_1, LOW);

  // Τώρα αλλάζεις σε INPUT για να διαβάσεις το echo
  pinMode(TRIG_ECHO_PIN_1, INPUT);
  duration = pulseIn(TRIG_ECHO_PIN_1, HIGH, 30000);  // timeout 30ms

  // Υπολογισμός απόστασης
  distance = (duration * 0.0343) / 2;

  Serial.print("Distance Front: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(10);
  

  pinMode(TRIG_ECHO_PIN_2, OUTPUT);
  digitalWrite(TRIG_ECHO_PIN_2, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_ECHO_PIN_2, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_ECHO_PIN_2, LOW);

  // Τώρα αλλάζεις σε INPUT για να διαβάσεις το echo
  pinMode(TRIG_ECHO_PIN_2, INPUT);
  duration = pulseIn(TRIG_ECHO_PIN_2, HIGH, 30000);  // timeout 30ms

  // Υπολογισμός απόστασης
  distance = (duration * 0.0343) / 2;

  Serial.print("Distance Back: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(10);


  pinMode(TRIG_ECHO_PIN_3, OUTPUT);
  digitalWrite(TRIG_ECHO_PIN_3, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_ECHO_PIN_3, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_ECHO_PIN_3, LOW);

  // Τώρα αλλάζεις σε INPUT για να διαβάσεις το echo
  pinMode(TRIG_ECHO_PIN_3, INPUT);
  duration = pulseIn(TRIG_ECHO_PIN_3, HIGH, 30000);  // timeout 30ms

  // Υπολογισμός απόστασης
  distance = (duration * 0.0343) / 2;

  Serial.print("Distance Right: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(10);


  pinMode(TRIG_ECHO_PIN_4, OUTPUT);
  digitalWrite(TRIG_ECHO_PIN_4, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_ECHO_PIN_4, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_ECHO_PIN_4, LOW);

  // Τώρα αλλάζεις σε INPUT για να διαβάσεις το echo
  pinMode(TRIG_ECHO_PIN_4, INPUT);
  duration = pulseIn(TRIG_ECHO_PIN_4, HIGH, 30000);  // timeout 30ms

  // Υπολογισμός απόστασης
  distance = (duration * 0.0343) / 2;

  Serial.print("Distance Left: ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(10);
}