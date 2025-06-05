#include <ESP32Servo.h>
int buttonPin = 2;
int buttonState = 0;
int loopNum = 1;
int motorPower = 1000;
//int prevButtonState = 0;

Servo servoMotor[4];            // Create an object for each servo
int servoPin[4] = {9,0,0,0}; // ESP32 pins to be used, starting from top right motor clockwise
int motorInput[4]= {0};

int startingDelay = 3000; // Time before loopNum starts (ms)

int motorUpdateSpeed = 4000;       // Minimum time (μs) between motor updates. 250 times a second, doesnt coincide much with pingSpeed
unsigned long lastMotorUpdate;

void setup() {

  pinMode(buttonPin, INPUT_PULLDOWN);

  Serial.begin(115200);

  // Initialise Motors
  servoMotor[0].attach(servoPin[0],1000,2000);  // Attaches the servos on each ESP32 pin
  servoMotor[0].write(90); // Provides a "neutral" pulse. The ESC won't start without this.
  
  delay(startingDelay);
  servoMotor[0].write(0);
}

void loop() {
  buttonState = digitalRead(buttonPin);

  if ( buttonState == HIGH ) {
    motorPower += 100;
    motorPower = constrain(motorPower,1000,2000);
    servoMotor[0].write(motorPower);
  }

  if ( buttonState == LOW ) {
    motorPower -= 100;
    motorPower = constrain(motorPower,1000,2000);
    servoMotor[0].write(motorPower);
  }

  Serial.print("Loop: ");  Serial.print(loopNum);  Serial.print("\t");  Serial.print("Motor Power: ");  Serial.print(motorPower);

  delay(1000);
  
  Serial.println();
  loopNum++;
}