#include <ESP32Servo.h>

int servoPin[4] = {2,15,8,4};  // ESP32 pins to be used
int startingDelay = 3000;
int percentVoltage = 0;
int power = 0;

Servo servoMotor[4]; // Starting from top left motor, clockwise

void setup() {
  Serial.begin(115200);

  for (int i=0; i<4; i++) {
    servoMotor[i].attach(servoPin[i],1000,2000);  // Attaches the servos on each ESP32 pin
    power = 90;
    servoMotor[i].write(power); // 0 - 180 -> 0% - 100% // Provides a "neutral" pulse. The ESC won't start without this.
    delay(100);
    servoMotor[i].write(0); // 0 - 180 -> 0% - 100% // Provides a "neutral" pulse. The ESC won't start without this.
  }

  delay(startingDelay);
  //Serial.print("Time ");
  //Serial.print("Gradient ");
  //Serial.print("%Voltage ");
  //Serial.println();
}

void loop() {
  Serial.println(power);
  for (int i=0; i<4; i++) {
    power = 1150;
    servoMotor[i].write(power);
  }
  delay(startingDelay);
  for (int i=0; i<4; i++) {
    power = 2000;
    servoMotor[i].write(power);
  }
  delay(startingDelay);
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