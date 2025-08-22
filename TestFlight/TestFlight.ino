#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>

//--- VAR DESCRIPTIONS ---//
// General

const int startingDelay = 3000;                                   // Time before loop starts (ms)
unsigned long loopNumber = 0;                                     // Loop number
uint8_t controllerMAC[] = { 0x78, 0x42, 0x1c, 0x1b, 0x25, 0x5c }; // MAC address of the controller to allow for ESPnow connection
bool emergencyShutdown = false;                                   // For the whoopsies
bool shutdown = false;                                            // For the not so bad whoopsies
#define radToDeg 57.29577951308232f

// Ali Motors

Servo servoMotor[4];                       // Create an object for each servo
const int servoPin[4] = { 2, 25, 26, 14 }; // ESP32 pins to be used, starting from top right motor clockwise // PIN 9 is probably broken
int motorInput[4] = { 0, 0, 0, 0 };        // 0 - 180 (Motor Degrees) OR 1000-2000 (μs) -> 0% - 100% Total Power Output of Motor

const int motorUpdateSpeed = 4000;     // Minimum time (μs) between motor updates. 250 times a second, doesnt coincide much with pingSpeed
unsigned long lastMotorUpdate = 0;     // Shows the last time the motors updated their speed (μs)
unsigned long motorUpdateDuration = 0; // Stores the duration between PID checks (μs)
double motorUpdateDurationSeconds = 0; // Stores the duration between PID checks (s)

// PID

bool PIDdisabled = false;

int throttleInput = 0;        // Extra power for every motor (μs)
const int noPower = 1000;     // Smallest PWM signal for no power (μs)
const int fullPower = 2000;   // Largest PWM signal for full power (μs)
const int minThrottle = 1180; // Throttle needed before motor shuts down (μs)
const int maxThrottle = 1800; // Max throttle to allow extra power for Roll, Pitch, Yaw (μs)

int PIDoutput[3] = { 0, 0, 0 };  // PID output for each motor for Roll, Pitch, Yaw (μs)

//const float constP[3] = {0.6,0.6,2};   // P for Roll, Pitch, Yaw
//const float constI[3] = {3.5,3.5,12};  // I for Roll, Pitch, Yaw
//const float constD[3] = {0.03,0.03,0}; // D for Roll, Pitch, Yaw

//const float constP[3] = { 14.370626521756, 14.370626521756, -0.566885796597361 };   // P for Roll, Pitch, Yaw
//const float constI[3] = { 32.1772873214827, 32.1772873214827, -1.04407391733098 };  // I for Roll, Pitch, Yaw
//const float constD[3] = { 1.01300889938204, 1.01300889938204, -0.041222921811501 }; // D for Roll, Pitch, Yaw

// Simulated pid (11/08/2025)
const float constP[3] = {0.0272431956712961, 0.0272431956712961, 0.00332236778236568};   // P for Roll, Pitch, Yaw
const float constI[3] = {0.0157983968092129, 0.0157983968092129, 0.00131008186463883};  // I for Roll, Pitch, Yaw
const float constD[3] = {0.0036575184260647, 0.0036575184260647, 0.00017080647458061}; // D for Roll, Pitch, Yaw

float desiredRate[3] = { 0, 0, 0 };  // Desired rate of Roll, Pitch, Yaw
float desiredRatePWM[3] = { 0, 0, 0 };  // Desired rate of Roll, Pitch, Yaw (μs)
float currentError[3] = { 0, 0, 0 }; // Error rate of Roll, Pitch, Yaw (μs)
float inputRate[3] = { 0, 0, 0 };    // Input rate of Roll, Pitch, Yaw, Thrust from accelerometer/gyroscope (μs)
float prevError[3] = { 0, 0, 0 };    // Previous error rate of Roll, Pitch, Yaw (μs)
float prevIterm[3] = { 0, 0, 0 };    // Sum of errors so far for rate of Roll, Pitch, Yaw (μs)

int integralWindupLimitMax = 400; // Limit past corrections to prevent overshoot (μs)
int integralWindupLimitMin = -400; // Limit past corrections to prevent overshoot (μs)

const float constAngleP = 2; // P for Roll and Pitch angles
const float constAngleI = 0; // I for Roll and Pitch angles
const float constAngleD = 0; // D for Roll and Pitch angles

float desiredAngle[2] = { 0, 0 };      // Desired angle of Roll and Pitch (μs)
float currentAngleError[2] = { 0, 0 }; // Error angle of Roll and Pitch (μs)
//inputAngleRate is just the kalmanAngle
float prevAngleError[2] = { 0, 0 }; // Error angle of Roll and Pitch (μs)
float prevAngleIterm[2] = { 0, 0 }; // Error angle of Roll and Pitch (μs)

// Kalman Filter

float kalmanAngle[2] = { 0, 0 };         // Kalman angle of Roll and Pitch starting at 0° (level takeoff) (μs)
float kalmanUncertainty[2] = { 2 * 2, 2 * 2 }; // Starting estimated error of Roll and Pitch at 2°
float angle[2] = { 0, 0 };                     // Angle of Roll and Pitch (°)
float kalmanAnglePWM[2] = {1500, 1500};// Kalman angle of Roll and Pitch (μs)

// GY-521 Gyroscope

Adafruit_MPU6050 mpu;  // Create the gyroscope object

float tempGyro = 0;  // Temperature of the Gyroscope (Unreliable for general calculations)

float gyroCalibrationX = 0;  //
float gyroCalibrationY = 0;  // Stores the values created during the calibration (°/s)
float gyroCalibrationZ = 0;  //

const int gyroCalibrationTests = 2000;  // How many measurements will be used to initialise the gyroscope

float gyroX = 0;  //
float gyroY = 0;  // Rotational Velocity (°/s)
float gyroZ = 0;  //

float accelerometerCalibrationX = -0.814; //
float accelerometerCalibrationY = 0.069;  // Fixed values expressing the sensor's slant
float accelerometerCalibrationZ = -1.096; //

float accelerometerX = 0;  //
float accelerometerY = 0;  // Linear Acceleration (m/s)
float accelerometerZ = 0;  //

//  ESP-Now Communication

struct dataIn { // Packet sent from the controller
  bool emergencyShutdown = 0;
  bool shutdown;
  int movementCommand[4] = { 0, 0, 0, 0 };
} controllerInstructions;

struct dataOut { // Packet sent to the controller
  float kalmanAngle[2] = { 0, 0 };
  float inputRateYaw = 0;
  float motorInput[4] = { 0, 0, 0, 0 };
  float randomData[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
} controllerData;

esp_now_peer_info_t peerInfo;

//--- PROGRAM START ---//

void setup() {
  Serial.begin(115200);
  Wire.begin();

  initialiseESPnow();

  initialiseMotors();

  initialiseGyro();

  Serial.println("Setup Complete");
  delay(startingDelay);
}

void loop() {
  Serial.println(loopNumber);

  loopESPnow();

  if (emergencyShutdown == true) {
    PIDdisabled = true;
    for (uint8_t ji = 0; ji < 2; ji++) {  // For Roll and Pitch
      prevAngleError[ji] = 0;
      prevAngleIterm[ji] = 0;
    }
    for (uint8_t j = 0; j < 3; j++) {  // For Roll, Pitch, Yaw
      prevError[j] = 0;
      prevIterm[j] = 0;
    }
    for (uint8_t i = 0; i < 4; i++) {
      servoMotor[i].write(0);
    }
  }

  if (shutdown == false ) {
    motorUpdateDurationSeconds = 0.004;
    getGyro();
    delay(4);
  }

  if (shutdown == true) {
    //for (uint8_t i = 1; i < 4; i++) {
    //  controllerInstructions.movementCommand[i] = 1500;
    //}
    //controllerInstructions.movementCommand[0] -= 100;
    stabiliseModeFlightControllerLoop();
  }

  //stabiliseModeFlightControllerLoop();

  loopNumber++;
}

//--- FUNCTIONS ---//

// Setup

void initialiseESPnow() {
  WiFi.mode(WIFI_STA);  // Set device as a Wi-Fi Station

  if (esp_now_init() != ESP_OK) {  // Init ESP-NOW
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);  // Register for a callback function that will be called when data is sent

  memcpy(peerInfo.peer_addr, controllerMAC, 6);  // Register peer
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {  // Add peer
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));  // Register for a callback function that will be called when data is received
}

void initialiseMotors() {  // Initialise Motors
  for (uint8_t i = 0; i < 4; i++) servoMotor[i].attach(servoPin[i], 1000, 2000);  // Attaches the servos on each ESP32 pin
  for (uint8_t i = 0; i < 4; i++) servoMotor[i].write(90);  // Provides a "neutral" pulse. The ESC won't start without this.
  delay(2000);
  for (uint8_t i = 0; i < 4; i++) servoMotor[i].write(0);
}

void initialiseGyro() {
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  //
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);       // Setup the MPU6050
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);     //

  gyroCalibrationX = 0;
  gyroCalibrationY = 0;
  gyroCalibrationZ = 0;

  for (int rateCalibrationAmount = 0; rateCalibrationAmount < gyroCalibrationTests; rateCalibrationAmount++) {  // Calibrate the gyroscope (drone needs to not move)
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
  for (uint8_t ji = 0; ji < 2; ji++) controllerData.kalmanAngle[ji] = kalmanAngle[ji];
  controllerData.inputRateYaw = inputRate[2];
  for (uint8_t i = 0; i < 4; i++) controllerData.motorInput[i] = motorInput[i];

  controllerData.randomData[0] = gyroX;
  controllerData.randomData[1] = gyroY;
  controllerData.randomData[2] = gyroZ;
  controllerData.randomData[3] = accelerometerX;
  controllerData.randomData[4] = accelerometerY;
  controllerData.randomData[5] = accelerometerZ;

  controllerData.randomData[6] = PIDoutput[0];
  controllerData.randomData[7] = PIDoutput[1];
  controllerData.randomData[8] = PIDoutput[2];

  controllerData.randomData[9] = throttleInput;
  controllerData.randomData[10] = desiredAngle[0];
  controllerData.randomData[11] = desiredAngle[1];
  controllerData.randomData[12] = desiredRate[2];

  controllerData.randomData[13] = emergencyShutdown;
  controllerData.randomData[14] = shutdown;

  esp_err_t result = esp_now_send(controllerMAC, (uint8_t *)&controllerData, sizeof(controllerData));  // Send message via ESP-NOW
}

void stabiliseModeFlightControllerLoop() {
  motorUpdateDuration = micros() - lastMotorUpdate;
  if (motorUpdateDuration >= motorUpdateSpeed && PIDdisabled == false) {
    lastMotorUpdate = micros();
    motorUpdateDurationSeconds = motorUpdateDuration / 1000000.0;

    getGyro();  // Get gyroX,Y,Z and kalmanAnglePWM for Roll and Pitch

    throttleInput = controllerInstructions.movementCommand[0];
    desiredAngle[0] = controllerInstructions.movementCommand[1];
    desiredAngle[1] = controllerInstructions.movementCommand[2];

    for (uint8_t ji = 0; ji < 2; ji++) {  // For Roll and Pitch
      currentAngleError[ji] = desiredAngle[ji] - kalmanAnglePWM[ji];

      float P, I, D;
      P = constAngleP * currentAngleError[ji];

      /* constAngleI and constAngleD are 0
      I = prevAngleIterm[ji] + constAngleI * (prevAngleError[ji] + currentAngleError[ji]) * motorUpdateDurationSeconds / 2;
      I = constrain(I, integralWindupLimitMin, integralWindupLimitMax);

      D = constAngleD * (currentAngleError[ji] - prevAngleError[ji]) / motorUpdateDurationSeconds;

      prevAngleError[ji] = currentAngleError[ji];
      prevAngleIterm[ji] = I;

      desiredRate[ji] = P + I + D;
      */

      desiredRate[ji] = P;
      //desiredRate[ji] = constrain(desiredRate[ji], integralWindupLimitMin, integralWindupLimitMax);
      desiredRatePWM[ji] = mapf(desiredRate[ji], -1000.0f, 1000.0f, 1000.0f, 2000.0f);
      desiredRatePWM[ji] = constrain(desiredRatePWM[ji], 1000.0f, 2000.0f);
    }

    inputRate[0] = mapf(gyroX, -75.0f, 75.0f, 1000.0f, 2000.0f);  // Get Roll
    inputRate[1] = mapf(gyroY, -75.0f, 75.0f, 1000.0f, 2000.0f);  // Get Pitch
    inputRate[2] = mapf(gyroZ, -75.0f, 75.0f, 1000.0f, 2000.0f);  // Get Yaw

    desiredRate[2] = controllerInstructions.movementCommand[3];  // In theory, if these 3 are equal they eliminate each other's forces resulting in a stabilised system, except the needed throttle to go up or down
    desiredRatePWM[2] = desiredRate[2];

    for (uint8_t j = 0; j < 3; j++) {  // For Roll, Pitch, Yaw
      currentError[j] = desiredRatePWM[j] - inputRate[j];

      float P, I, D;
      P = constP[j] * currentError[j];

      I = prevIterm[j] + constI[j] * (prevError[j] + currentError[j]) * motorUpdateDurationSeconds / 2;
      I = constrain(I, integralWindupLimitMin, integralWindupLimitMax);

      D = constD[j] * (currentError[j] - prevError[j]) / motorUpdateDurationSeconds;

      PIDoutput[j] = P + I + D;
      PIDoutput[j] = constrain(PIDoutput[j], integralWindupLimitMin, integralWindupLimitMax);

      prevError[j] = currentError[j];
      prevIterm[j] = I;
    }
    if (throttleInput > maxThrottle) throttleInput = maxThrottle;

    motorInput[0] = throttleInput + PIDoutput[0] - PIDoutput[1] + PIDoutput[2];
    motorInput[1] = throttleInput + PIDoutput[0] + PIDoutput[1] - PIDoutput[2];
    motorInput[2] = throttleInput - PIDoutput[0] + PIDoutput[1] + PIDoutput[2];
    motorInput[3] = throttleInput - PIDoutput[0] - PIDoutput[1] - PIDoutput[2];

    for (uint8_t i = 0; i < 4; i++) {
      motorInput[i] = constrain(motorInput[i], minThrottle, 2000);
      servoMotor[i].write(motorInput[i]);
    }
  }
}

void getGyro() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  tempGyro = temp.temperature;

  gyroX = ( g.gyro.x - gyroCalibrationX )* radToDeg;  //
  gyroY = ( g.gyro.y - gyroCalibrationY )* radToDeg;  // Acceleration in °/s
  gyroZ = ( g.gyro.z - gyroCalibrationZ )* radToDeg;  //

  accelerometerX = a.acceleration.x + accelerometerCalibrationX;  //
  accelerometerY = a.acceleration.y + accelerometerCalibrationY;  // Acceleration in m/s
  accelerometerZ = a.acceleration.z + accelerometerCalibrationZ;  //

  angle[0] = atan(accelerometerY / sqrt(accelerometerX * accelerometerX + accelerometerZ * accelerometerZ)) / (3.142 / 180);
  angle[1] = atan(accelerometerX / sqrt(accelerometerY * accelerometerY + accelerometerZ * accelerometerZ)) / (3.142 / 180);

  //Serial.print(angle[0]);  Serial.print("\t");
  //Serial.println(angle[1]);

  for (uint8_t ji = 0; ji < 2; ji++) {  // For Roll and Pitch
    if (ji == 0) kalmanAngle[ji] += motorUpdateDurationSeconds * gyroX;
    else kalmanAngle[ji] += motorUpdateDurationSeconds * gyroY;
    kalmanUncertainty[ji] += motorUpdateDurationSeconds * motorUpdateDurationSeconds * 4 * 4;
    float kalmanGain = kalmanUncertainty[ji] / (kalmanUncertainty[ji] + 3 * 3);
    kalmanAngle[ji] += kalmanGain * (angle[ji] - kalmanAngle[ji]);
    kalmanUncertainty[ji] = (1 - kalmanGain) * kalmanUncertainty[ji];

    kalmanAnglePWM[ji] = mapf(kalmanAngle[ji], -50.0f, 50.0f, 1000.0f, 2000.0f);
  }

  //Serial.print(kalmanAngle[0]);  Serial.print("\t");
  //Serial.print(kalmanUncertainty[0]);  Serial.print("\t");
  //Serial.print(kalmanAngle[1]);  Serial.print("\t");
  //Serial.println(kalmanUncertainty[1]);
}

// Interrupts

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {  // Callback when data is sent
  if (status != 0) {
    Serial.println("Delivery Failed");
  }
}

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {  // Callback when data is received
  //Serial.println("Data Received");
  memcpy(&controllerInstructions, incomingData, sizeof(controllerInstructions));
  emergencyShutdown = controllerInstructions.emergencyShutdown;
  shutdown = controllerInstructions.shutdown;
  // This would be here but I place the struct vars directly in // for (uint8_t i=0; i<4; i++)
}

inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
