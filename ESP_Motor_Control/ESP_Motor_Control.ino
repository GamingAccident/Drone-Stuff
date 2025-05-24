#include <ESP32Servo.h>

int servoPin[4] = {25,26,27,9};  // ESP32 pins to be used
int startingDelay = 3000;
int percentVoltage = 0;

Servo servoMotor[4]; // Starting from top left motor, clockwise

void setup() {
  Serial.begin(115200);

  for (int i=0; i<4; i++) {
    servoMotor[i].attach(servoPin[i],1000,2000);  // Attaches the servos on each ESP32 pin
    servoMotor[i].write(90); // 0 - 180 -> 0% - 100% // Provides a "neutral" pulse. The ESC won't start without this.
  }

  delay(startingDelay);
  //Serial.print("Time ");
  Serial.print("Gradient ");
  Serial.print("%Voltage ");
  Serial.println();
}

void loop() {
  manualControl();
}

void manualControl() {
  int analogValueGradient = analogRead(34);
  int analogValueVoltage = analogRead(35);
  int currentTime = millis()-startingDelay;

  int percentVoltage = map (analogValueVoltage, 0, 4095, 0, 100); //Maps the signal pulse to a percentage
  int degreeVoltage = map (analogValueVoltage, 0, 4095, 0, 180);  //Maps the signal pulse to degrees

  Serial.print(currentTime);
  Serial.print(",");
  Serial.print(analogValueGradient);
  Serial.print(",");
  Serial.print(percentVoltage);

  for (int i=0; i<4; i++) {
    servoMotor[i].write(degreeVoltage);
  }

  Serial.println();
  delay(100);
}

void thrustMeasurement() {
  int analogValueGradient = analogRead(34);
  percentVoltage ++;
  
  int degreeVoltage = map (percentVoltage, 0, 100, 0, 180);  // Might want finer control

  Serial.print(analogValueGradient);
  Serial.print(",");
  Serial.print(percentVoltage);

  for (int i=0; i<4; i++) {
    servoMotor[i].write(degreeVoltage);
  }

  Serial.println();
  delay(100);
}