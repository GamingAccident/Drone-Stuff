#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

// General

int startingDelay = 3000; // Time before loop starts (ms)
unsigned long k;          // Loop number

// Ali Motors

Servo servoMotor[4];            // Create an object for each servo
int servoPin[4] = {25,26,27,9}; // ESP32 pins to be used, starting from top right motor clockwise
int motorInput[4]= {0};         // 0 - 180 (Motor Degrees) OR 1000-2000 (μs) -> 0% - 100% Total Power Output of Motor

int motorUpdateSpeed = 4000;       // Minimum time (μs) between motor updates. 250 times a second, doesnt coincide much with pingSpeed
unsigned long lastMotorUpdate;     // Shows the last time the motors updated their speed (μs)
unsigned long motorUpdateDuration; // Stores the duration between PID checks (μs)
double motorUpdateDurationSeconds; // Stores the duration between PID checks (s)

// PID

bool PIDdisabled = true;

int throttleInput = 0;  // Extra power for every motor (μs)
int minThrottle = 1180; // Throttle needed before motor shuts down (μs)
int maxThrottle = 1800; // Max throttle to allow extra power for Roll, Pitch, Yaw (μs)

int PIDoutput[3] = {0}; // PID output for each motor for Roll, Pitch, Yaw (μs)

float constP[3] = {0.6,0.6,2};   //TBD // P for Roll, Pitch, Yaw
float constI[3] = {3.5,3.5,12};  //TBD // I for Roll, Pitch, Yaw
float constD[3] = {0.03,0.03,0}; //TBD // D for Roll, Pitch, Yaw

float desiredRate[3] = {0};  // Desired rate of Roll, Pitch, Yaw (μs)
float currentError[3] = {0}; // Error rate of Roll, Pitch, Yaw (μs)
float inputRate[3] = {0};    // Input rate of Roll, Pitch, Yaw, Thrust from accelerometer/gyroscope (μs)
float prevError[3] = {0};    // Previous error rate of Roll, Pitch, Yaw (μs)
float prevIterm[3] = {0};    // Sum of errors so far for rate of Roll, Pitch, Yaw (μs)

int integralWindupLimit = 400; // Limit past corrections to prevent overshoot (μs)

// GY-521 Gyroscope

Adafruit_MPU6050 mpu; // Create the gyroscope object

float tempGyro;

float gyroCalibrationX; //
float gyroCalibrationY; // Stores the values created during the calibration (°/s)
float gyroCalibrationZ; //

int gyroCalibrationTests = 2000; // How many measurements will be used to initialise the gyroscope

float gyroX; //
float gyroY; // Rotational Velocity (°/s)
float gyroZ; //

float accelerometerCalibrationX = -0.05; //
float accelerometerCalibrationY = 0.01;  // Fixed values expressing the sensor's slant // TBD Change them when fitting the sensor in new chassis
float accelerometerCalibrationZ = -0.01; //

float accelerometerX; //
float accelerometerY; // Linear Acceleration (g)
float accelerometerZ; //

// Web Server

const char* ssid = "YourNetworkName";  // Enter SSID here // TBD
const char* password = "YourPassword";  //Enter Password here // TBD

void setup() {
  Serial.begin(115200);

  serverSetup();

  for (uint8_t i=0; i<4; i++) { // Initialise Motors
    servoMotor[i].attach(servoPin[i],1000,2000);  // Attaches the servos on each ESP32 pin
    servoMotor[i].write(90); // Provides a "neutral" pulse. The ESC won't start without this.
  }

  initialiseGyro();

  delay(startingDelay);
}

void loop() {

  server.handleClient();

  motorUpdateDuration = micros() - lastMotorUpdate;
  if (motorUpdateDuration >= motorUpdateSpeed && PIDdisabled == false) {
    lastMotorUpdate = micros();

    getGyro(); // Get gyroX,Y,Z and accelerometerX,Y,Z

    // map(acceleration,-75,75,1000,2000); Drone angle to μs
    inputRate[0] = 20/3*gyroX+1500; // Get Roll
    inputRate[1] = 20/3*gyroY+1500; // Get Pitch
    inputRate[2] = 20/3*gyroZ+1500; // Get Yaw

    throttleInput = 0;
    desiredRate[0] = 1500; //
    desiredRate[1] = 1500; // In theory, if these 3 are equal they eliminate each other's forces resulting in a stabilised system, except the needed throttle to go up or down
    desiredRate[2] = 1500; //

    for (uint8_t j = 0; j < 3; j++) { // For Roll, Pitch, Yaw
      currentError[j] = desiredRate[j]-inputRate[j];

      float P,I,D;
      motorUpdateDurationSeconds = motorUpdateDuration/1000000.0;
      P = constP[j]*currentError[j];

      I = prevIterm[j]+constI[j]*(prevError[j]+currentError[j])*motorUpdateDurationSeconds/2;
      if (I > integralWindupLimit) I = integralWindupLimit;
      else if (I < -integralWindupLimit) I = -integralWindupLimit;
      
      D = constD[j]*(currentError[j]-prevError[j])/motorUpdateDurationSeconds;

      PIDoutput[j] = P+I+D;
      if (PIDoutput[j] > integralWindupLimit) PIDoutput[j] = integralWindupLimit;
      else if (PIDoutput[j] < -integralWindupLimit) PIDoutput[j] = -integralWindupLimit;

      prevError[j] = currentError[j];
      prevIterm[j] = I;
    }
    if (throttleInput > maxThrottle) throttleInput = maxThrottle;

    motorInput[0] = throttleInput-PIDoutput[0]-PIDoutput[1]-PIDoutput[2];
    motorInput[1] = throttleInput+PIDoutput[0]-PIDoutput[1]+PIDoutput[2]; // TBD Review these before flying (EP 11)
    motorInput[2] = throttleInput+PIDoutput[0]+PIDoutput[1]-PIDoutput[2];
    motorInput[3] = throttleInput-PIDoutput[0]+PIDoutput[1]+PIDoutput[2];

    for (uint8_t i=0; i<4; i++) {
      if (motorInput[i] > 2000) motorInput[i] = 2000;
      else if (motorInput[i] < minThrottle) motorInput[i] = minThrottle;
      servoMotor[i].write(motorInput[i]);
    }
  }
  
  k++;
}

// Server

void serverSetup() {
  Serial.println("Connecting to ");
  Serial.println(ssid);

  //connect to your local wi-fi network
  WiFi.begin(ssid, password);

  //check wi-fi is connected to wi-fi network
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected..!");
  
  // Initialize mDNS
  if (!MDNS.begin("esp32")) {   // Set the hostname to "esp32.local"
    Serial.println("Error setting up MDNS responder!");
    while(1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");
  
  server.on("/", handle_OnConnect);
  server.onNotFound(handle_NotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void handle_OnConnect() {
  server.send(200, "text/html", "<!DOCTYPE html><html><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\"></head><body><h1>Hey there!</h1></body></html>"); 
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

// PID

void resetPID() {
  for (uint8_t j = 0; j < 3; j++) { // For Roll, Pitch, Yaw
    prevError[j] = 0;
    prevIterm[j] = 0;
  }
}

void hardStop() {
  PIDdisabled = true;
  resetPID();
  for (uint8_t i=0; i<4; i++) {
    servoMotor[i].write(0);
  }
}

void softStop() {
  throttleInput -= 10;
}

// Gyroscope

void initialiseGyro() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  gyroCalibrationX = 0;
  gyroCalibrationY = 0;
  gyroCalibrationZ = 0;

  for ( int rateCalibrationAmount = 0; rateCalibrationAmount<gyroCalibrationTests; rateCalibrationAmount++) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    gyroCalibrationX += g.gyro.x;
    gyroCalibrationY += g.gyro.y;
    gyroCalibrationZ += g.gyro.z;
  }

  gyroCalibrationX /= gyroCalibrationTests;
  gyroCalibrationY /= gyroCalibrationTests;
  gyroCalibrationZ /= gyroCalibrationTests;
}

void getGyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  tempGyro = temp.temperature;
  
  gyroX = g.gyro.x-gyroCalibrationX;
  gyroY = g.gyro.y-gyroCalibrationY;
  gyroZ = g.gyro.z-gyroCalibrationZ;

  accelerometerX = a.acceleration.x-accelerometerCalibrationX;
  accelerometerY = a.acceleration.y-accelerometerCalibrationY;
  accelerometerZ = a.acceleration.z-accelerometerCalibrationZ;
}
