#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// General

int startingDelay = 3000; // Time before loop starts

// Ali Servo

int servoPin[4] = {25,26,27,9};  // ESP32 pins to be used for the motors
Servo servoMotor[4];             // Starting from top left motor, clockwise, create the motor objects

int percentVoltage = 0; // Initialisation of %voltage given to the motors

// GY-521 Gyroscope

Adafruit_MPU6050 mpu; // Create the gyroscope object

float tempGyro;

float rateCalibrationX; //
float rateCalibrationY; // Used to initialise the gyroscope
float rateCalibrationZ; //

int rateCalibrationMax = 2000; // How many measurements will be used to initialise the gyroscope

float calibrationX = -0.05; //
float calibrationY = 0.01;  // Fixed values expressing the sensor's slant // TBD Change them when fitting the sensor in new chassis
float calibrationZ = -0.01; //

float accelerationX; //
float accelerationY; // Acceleration per axis
float accelerationZ; //

float gyroX; //
float gyroY; // Speed per axis
float gyroZ; //

// PID Controller

unsigned long lastTime;
float Input, Output, Setpoint;
float errSum, lastErr;
float kp = 0.6;
float ki = 3.5;
float kd = 0.03;
float ratee;

void setup() {
  Serial.begin(115200);

  for (int i=0; i<4; i++) {
    servoMotor[i].attach(servoPin[i],1000,2000);     // Attaches the servos on each ESP32 pin
    servoMotor[i].write(90); // 0 - 180 -> 0% - 100% // Provides a "neutral" pulse. The ESC won't start without this.
  }

  initialiseGyro();

  delay(startingDelay);
  //Serial.print("Time ");
  //Serial.print("Gradient ");
  //Serial.print("%Voltage ");
  //Serial.println();
  
}

void loop() {
  oneMotorControl ();
}
void oneMotorControl () {
  int analogValueGradient = analogRead(34);

  Setpoint = 1360;
  Input = analogValueGradient;

  compute();
  
  Serial.print("Rate: ");
  Serial.print(ratee);
  Serial.print("\t");
  Serial.print("Gradient");
  Serial.print(analogValueGradient);
  Serial.print("\t");
  Serial.print("Output: ");
  Serial.println(Output);

  delay(100);
}

void initialiseGyro() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  for ( int rateCalibrationAmount = 0; rateCalibrationAmount<rateCalibrationMax; rateCalibrationAmount++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    rateCalibrationX += a.acceleration.x;
    rateCalibrationY += a.acceleration.y;
    rateCalibrationZ += a.acceleration.z;
  }

  rateCalibrationX /= rateCalibrationMax;
  rateCalibrationY /= rateCalibrationMax;
  rateCalibrationZ /= rateCalibrationMax;
}

void getGyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  tempGyro = temp.temperature;

  accelerationX = a.acceleration.x-rateCalibrationX;
  accelerationY = a.acceleration.y-rateCalibrationY;
  accelerationZ = a.acceleration.z-rateCalibrationZ;

  gyroX = g.gyro.x-calibrationX;
  gyroY = g.gyro.y-calibrationY;
  gyroZ = g.gyro.z-calibrationZ;
}

void manualControl() {
  int analogValueGradient = analogRead(34);
  int analogValueVoltage = analogRead(35);
  int currentTime = millis()-startingDelay;

  int percentVoltage = map (analogValueVoltage, 0, 4095, 0, 100); //Maps the signal pulse to a percentage
  int degreeVoltage = map (analogValueVoltage, 0, 4095, 0, 180);  //Maps the signal pulse to degrees

  //Serial.print(currentTime);
  //Serial.print(",");
  //Serial.print(analogValueGradient);
  //Serial.print(",");
  //Serial.print(percentVoltage);
  //Serial.println();

  for (int i=0; i<4; i++) {
    servoMotor[i].write(degreeVoltage);
  }
}

void compute() {
  /*How long since we last calculated*/
  unsigned long now = millis();

  float timeChange = (float)(now - lastTime);
  
  /*Compute all the working error variables*/
  float error = Setpoint - Input;
  errSum += (error * timeChange);
  float dErr = (error - lastErr) / timeChange;
  
  /*Compute PID Output*/
  Output = kp * error + ki * errSum + kd * dErr;
  
  /*Remember some variables for next time*/
  ratee = error-lastErr;
  lastErr = error;
  lastTime = now;
}