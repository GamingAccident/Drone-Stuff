#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

//--- VAR DESCRIPTIONS ---//
// General

int startingDelay = 3000;       // Time before loop starts (ms)
unsigned long loopNumber;       // Loop number
uint8_t controllerMAC[] = {0x78,0x42,0x1c,0x1b,0x25,0x5c}; // MAC address of the controller to allow for ESPnow connection
bool emergencyShutdown = false; // For the whoopsies

// Ali Motors

Servo servoMotor[4];            // Create an object for each servo
int servoPin[4] = {25,26,27,9}; // ESP32 pins to be used, starting from top right motor clockwise
int motorInput[4]= {0};         // 0 - 180 (Motor Degrees) OR 1000-2000 (μs) -> 0% - 100% Total Power Output of Motor

int motorUpdateSpeed = 4000;       // Minimum time (μs) between motor updates. 250 times a second, doesnt coincide much with pingSpeed
unsigned long lastMotorUpdate;     // Shows the last time the motors updated their speed (μs)
unsigned long motorUpdateDuration; // Stores the duration between PID checks (μs)
double motorUpdateDurationSeconds; // Stores the duration between PID checks (s)

// PID

bool PIDdisabled = false;

int throttleInput = 0;  // Extra power for every motor (μs)
int noPower = 1000;     // Smallest PWM signal for no power (μs)
int fullPower = 2000;   // Largest PWM signal for full power (μs)
int minThrottle = 1180; // Throttle needed before motor shuts down (μs)
int maxThrottle = 1800; // Max throttle to allow extra power for Roll, Pitch, Yaw (μs)

int PIDoutput[3] = {0}; // PID output for each motor for Roll, Pitch, Yaw (μs)

float constP[3] = {0.6,0.6,2};   //TBD // P for Roll, Pitch, Yaw
float constI[3] = {3.5,3.5,12};  //TBD // I for Roll, Pitch, Yaw
float constD[3] = {0.03,0.03,0}; //TBD // D for Roll, Pitch, Yaw

int desiredRate[3] = {0};    // Desired rate of Roll, Pitch, Yaw (μs)
float currentError[3] = {0}; // Error rate of Roll, Pitch, Yaw (μs)
float inputRate[3] = {0};    // Input rate of Roll, Pitch, Yaw, Thrust from accelerometer/gyroscope (μs)
float prevError[3] = {0};    // Previous error rate of Roll, Pitch, Yaw (μs)
float prevIterm[3] = {0};    // Sum of errors so far for rate of Roll, Pitch, Yaw (μs)

int integralWindupLimit = 400; // Limit past corrections to prevent overshoot (μs)

// Kalman Filter

float kalmanAngle[2] = {0}; // Kalman angle of Roll and Pitch starting at 0° (level takeoff)
float kalmanUncertainty[2] = {2*2,2*2}; // Starting estimated error of Roll and Pitch at 2°
float angle[2]; // Angle of Roll and Pitch (°)

// GY-521 Gyroscope

Adafruit_MPU6050 mpu; // Create the gyroscope object

float tempGyro; // Temperature of the Gyroscope (Unreliable for general calculations)

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
float accelerometerY; // Linear Acceleration (m/s)
float accelerometerZ; //

//  ESP-Now Communication

struct dataIn { // Packet sent from the controller
  bool emergencyShutdown;
  int throttleInput;
  int desiredRate[3];
} controllerInstructions;

struct dataOut { // Packet sent to the controller
  // TBD all other sensor data
  float gyroX;
  float gyroY;
  float gyroZ;
  float accelerometerX;
  float accelerometerY;
  float accelerometerZ;
} controllerData;

esp_now_peer_info_t peerInfo;

//--- PROGRAM START ---//

void setup() {
  Serial.begin(115200);

  //initialiseESPnow();

  initialiseMotors();

  initialiseGyro();

  Serial.println("Setup Complete");
  delay(startingDelay);
}

void loop() {
  //Serial.println(loopNumber);

  //loopESPnow();

  flightControllerLoop();
  
  loopNumber++;
}

//--- FUNCTIONS ---//

// Setup

void initialiseESPnow() {
  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station

  if (esp_now_init() != ESP_OK) { // Init ESP-NOW
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // Get the status of transmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, controllerMAC, 6); // Register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){ // Add peer  
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv)); // Register for a callback function that will be called when data is received
}

void initialiseMotors() {
  for (uint8_t i=0; i<4; i++) { // Initialise Motors
    servoMotor[i].attach(servoPin[i],1000,2000);  // Attaches the servos on each ESP32 pin
    servoMotor[i].write(90); // Provides a "neutral" pulse. The ESC won't start without this.
  }
}

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

// Loop

void loopESPnow() {
  esp_err_t result = esp_now_send(controllerMAC, (uint8_t *) &controllerData, sizeof(controllerData)); // Send message via ESP-NOW
}

void flightControllerLoop() {
  motorUpdateDuration = micros() - lastMotorUpdate;
  if (motorUpdateDuration >= motorUpdateSpeed && PIDdisabled == false) {
    lastMotorUpdate = micros();
    motorUpdateDurationSeconds = motorUpdateDuration/1000000.0;

    getGyro(); // Get gyroX,Y,Z and accelerometerX,Y,Z

    // map(acceleration,-75,75,1000,2000); Drone angle to μs
    inputRate[0] = 20/3*gyroX+1500; // Get Roll
    inputRate[1] = 20/3*gyroY+1500; // Get Pitch
    inputRate[2] = 20/3*gyroZ+1500; // Get Yaw

    //throttleInput = controllerInstructions.throttleInput;
    //desiredRate[0] = controllerInstructions.desiredRate[0]; // TBD
    //desiredRate[1] = controllerInstructions.desiredRate[1]; // In theory, if these 3 are equal they eliminate each other's forces resulting in a stabilised system, except the needed throttle to go up or down
    //desiredRate[2] = controllerInstructions.desiredRate[2]; //

    throttleInput = 0;
    desiredRate[0] = 1500; //
    desiredRate[1] = 1500; // In theory, if these 3 are equal they eliminate each other's forces resulting in a stabilised system, except the needed throttle to go up or down
    desiredRate[2] = 1500; //

    for (uint8_t j = 0; j < 3; j++) { // For Roll, Pitch, Yaw
      currentError[j] = desiredRate[j]-inputRate[j];

      float P,I,D;
      P = constP[j]*currentError[j];

      I = prevIterm[j]+constI[j]*(prevError[j]+currentError[j])*motorUpdateDurationSeconds/2;
      constrain(I,-integralWindupLimit,integralWindupLimit);
      
      D = constD[j]*(currentError[j]-prevError[j])/motorUpdateDurationSeconds;

      PIDoutput[j] = P+I+D;
      constrain(PIDoutput[j],-integralWindupLimit,integralWindupLimit);

      prevError[j] = currentError[j];
      prevIterm[j] = I;
    }
    if (throttleInput > maxThrottle) throttleInput = maxThrottle;

    motorInput[0] = throttleInput-PIDoutput[0]-PIDoutput[1]-PIDoutput[2];
    motorInput[1] = throttleInput+PIDoutput[0]-PIDoutput[1]+PIDoutput[2]; // TBD Review these before flying (EP 11)
    motorInput[2] = throttleInput+PIDoutput[0]+PIDoutput[1]-PIDoutput[2];
    motorInput[3] = throttleInput-PIDoutput[0]+PIDoutput[1]+PIDoutput[2];

    for (uint8_t i=0; i<4; i++) {
      constrain(motorInput[i],minThrottle,2000);
      servoMotor[i].write(motorInput[i]);
    }
  }
}

void getGyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  tempGyro = temp.temperature;
  
  gyroX = g.gyro.x-gyroCalibrationX; // 
  gyroY = g.gyro.y-gyroCalibrationY; // Acceleration in °/s
  gyroZ = g.gyro.z-gyroCalibrationZ; // 

  accelerometerX = a.acceleration.x-accelerometerCalibrationX; //
  accelerometerY = a.acceleration.y-accelerometerCalibrationY; // Acceleration in m/s
  accelerometerZ = a.acceleration.z-accelerometerCalibrationZ; //

  //Serial.print(gyroX);  Serial.print("\t");
  //Serial.print(gyroY);  Serial.print("\t");
  //Serial.print(gyroZ);  Serial.print("\t");
  //Serial.print(accelerometerX);  Serial.print("\t");
  //Serial.print(accelerometerY);  Serial.print("\t");
  //Serial.print(accelerometerZ);  Serial.print("\t");
  //Serial.println();

  angle[0] = atan(accelerometerY/sqrt(accelerometerX*accelerometerX+accelerometerZ*accelerometerZ))/(3.142/180);
  angle[1] = atan(accelerometerX/sqrt(accelerometerY*accelerometerY+accelerometerZ*accelerometerZ))/(3.142/180);

  //Serial.print(angle[0]);  Serial.print("\t");
  //Serial.println(angle[1]);

  for (uint8_t ji = 0; ji < 2; ji++) { // For Roll and Pitch
    if (ji == 0) kalmanAngle[ji] += motorUpdateDurationSeconds*gyroX;
    else kalmanAngle[ji] += motorUpdateDurationSeconds*gyroY;
    kalmanUncertainty[ji] += motorUpdateDurationSeconds*motorUpdateDurationSeconds*4*4;
    float kalmanGain = kalmanUncertainty[ji]/(kalmanUncertainty[ji]+3*3);
    kalmanAngle[ji] += kalmanGain*(angle[ji]-kalmanAngle[ji]);
    kalmanUncertainty[ji] = (1-kalmanGain)*kalmanUncertainty[ji];

    //kalmanPrediction[ji] = kalmanAngle[ji];
    //kalmanPredictionUncertainty[ji] = kalmanUncertainty[ji];
  }

  //Serial.print(kalmanAngle[0]);  Serial.print("\t");
  //Serial.print(kalmanUncertainty[0]);  Serial.print("\t");
  //Serial.print(kalmanAngle[1]);  Serial.print("\t");
  //Serial.println(kalmanUncertainty[1]);
}

// Interrupts

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { // Callback when data is sent
  if (status != 0){
    Serial.println("Delivery Failed");
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) { // Callback when data is received
  Serial.println("Data Received");
  memcpy(&controllerInstructions, incomingData, sizeof(controllerInstructions));
  emergencyShutdown = controllerInstructions.emergencyShutdown;
  throttleInput = controllerInstructions.throttleInput;
  memcpy(desiredRate, controllerInstructions.desiredRate, sizeof(controllerInstructions.desiredRate));
}

// Unused

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
