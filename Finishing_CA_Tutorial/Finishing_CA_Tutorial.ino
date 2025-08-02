#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
#include <BasicLinearAlgebra.h>
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>

//--- VAR DESCRIPTIONS ---//
// General

const int startingDelay = 3000;                                   // Time before loop starts (ms)
unsigned long loopNumber = 0;                                     // Loop number
uint8_t controllerMAC[] = { 0x78, 0x42, 0x1c, 0x1b, 0x25, 0x5c }; // MAC address of the controller to allow for ESPnow connection
bool emergencyShutdown = false;                                   // For the whoopsies
bool shutdown = false;                                            // For the not so bad whoopsies

// AHT20+BMP280 OR 4566 Adafruit

Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;

const float speedOfSound = 331.3;    // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float tempModifier = 0.606;    // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float humidityModifier = 1.26; // https://sengpielaudio.com/calculator-airpressure.htm
float currentTemp = 25;              // Celcius degrees //TBD
float currentHumidity = 0.5;         // % Humidity //TBD
float currentAltitude = 0;           // Altitude (m) from ground //TBD
float altitudeCalibration = 0;       // True Altitude

// HC-SR04 SUPERSONIC SENSORS

const int sonarNumber = 6; // Number of supersonic sensors
const int pingSpeed = 50;  // Minimum ms between sensor pings. 50ms would be 20 times a second.

float echoDuration[sonarNumber];  // Measured distance in ms (Initialised in setup)
float finalDistance[sonarNumber]; // Final measured distance (cm)

const int sonarPin[sonarNumber] = { 0, 0, 0, 0, 0, 0 }; // Pin numbers for the supersonic sensors
int currentSonar = 5;                                     // Which sonar is awaiting input
bool cycleCompleted[sonarNumber] = { 0, 0, 0, 0, 0, 0 };  // Has this sonar received the echo (Initialised in setup)

unsigned long lastSonarPing = 0;       // Shows the last time any sensor sent out a ping (ms)
unsigned long echoStart = 0;           // Saves the time the echo starts (μs)
unsigned long sonarUpdateDuration = 0; // Stores the duration between supersonic checks (ms)

// Ali Motors

Servo servoMotor[4];                       // Create an object for each servo
const int servoPin[4] = { 2, 25, 4, 16 };  // ESP32 pins to be used, starting from top right motor clockwise
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

const float constP[3] = {0.6,0.6,2};   // P for Roll, Pitch, Yaw
const float constI[3] = {3.5,3.5,12};  // I for Roll, Pitch, Yaw
const float constD[3] = {0.03,0.03,0}; // D for Roll, Pitch, Yaw

//const float constP[3] = { 14.370626521756, 14.370626521756, -0.566885796597361 };   // P for Roll, Pitch, Yaw
//const float constI[3] = { 32.1772873214827, 32.1772873214827, -1.04407391733098 };  // I for Roll, Pitch, Yaw
//const float constD[3] = { 1.01300889938204, 1.01300889938204, -0.041222921811501 }; // D for Roll, Pitch, Yaw

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

float accelerometerCalibrationX = -0.3;  //
float accelerometerCalibrationY = -0.86; // Fixed values expressing the sensor's slant // TBD Change them when fitting the sensor in new chassis
float accelerometerCalibrationZ = -0.71; //

float accelerometerX = 0;  //
float accelerometerY = 0;  // Linear Acceleration (m/s)
float accelerometerZ = 0;  //

float verticalAcceleration = 0;
float verticalSpeed = 0;

// 2D Kalman Filter

using namespace BLA;
float altitudeKalman = 0;
float verticalVelocityKalman = 0;
BLA::Matrix<2,2>F;
BLA::Matrix<2,1>G;
BLA::Matrix<2,2>P;
BLA::Matrix<2,2>Q;
BLA::Matrix<2,1>S;
BLA::Matrix<1,2>H;
BLA::Matrix<2,2>I;
BLA::Matrix<1,1>Acc;
BLA::Matrix<2,1>K;
BLA::Matrix<1,1>R;
BLA::Matrix<1,1>L;
BLA::Matrix<1,1>M;

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
  float randomData[10] = {0,0,0,0,0,0,0,0,0,0};
} controllerData;

esp_now_peer_info_t peerInfo;

//--- PROGRAM START ---//

void setup() {
  Serial.begin(115200);
  Wire.begin();

  initialiseESPnow();

  //initialiseSuperDuperSonic();

  initialiseMotors();

  initialiseGyro();

  Serial.println("Setup Complete");
  delay(startingDelay);
}

void loop() {
  //Serial.println(loopNumber);

  loopESPnow();

  //Serial.println(emergencyShutdown);

  //superDuperSonicLoop();

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

  if (shutdown == true) {
    for (uint8_t i = 1; i < 4; i++) {
      controllerInstructions.movementCommand[i] = 1500;
    }
    controllerInstructions.movementCommand[0] -= 100;
  }

  stabiliseModeFlightControllerLoop();

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

void initialiseAdafruit() {
  if (!aht.begin()) { Serial.println("Failed to initialize AHT20 sensor!"); }

  if (!bmp.begin(0x77)) {  // Default I²C address for BMP280 is 0x76
    Serial.println("Failed to initialize BMP280 sensor!");
  }

  for (int rateCalibrationAmount = 0; rateCalibrationAmount < gyroCalibrationTests; rateCalibrationAmount++) {  // Calibrate the altitude (drone needs to not move) // Same amount of tests as gyro
    float altitudeCalibration += bmp.readAltitude(1013.25); // Sea level pressure in hPa
  }

  altitudeCalibration /= gyroCalibrationTests
  gyroCalibrationX /= gyroCalibrationTests;
}

void initialiseGyro() {
  if (!mpu.begin()) { Serial.println("Failed to find MPU6050 chip"); }

  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);  //
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);        // Setup the MPU6050
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);     //

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

  F = {1,motorUpdateDurationSeconds,0,1};
  G = {0.5*motorUpdateDurationSeconds*motorUpdateDurationSeconds,motorUpdateDurationSeconds};
  H = {1,0};
  I = {1,0,0,1};
  Q = G *~ G*100;
  R = {900};
  P = {0,0,0,0};
  S = {0,0};
}

void initialiseSuperDuperSonic() {  // Sets the first value of the supersonics to 69cm
  for (uint8_t i = 0; i < sonarNumber; i++) {
    echoDuration[i] = 3976.03;
    cycleCompleted[i] = true;
  }
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

  esp_err_t result = esp_now_send(controllerMAC, (uint8_t *)&controllerData, sizeof(controllerData));  // Send message via ESP-NOW
}

void superDuperSonicLoop() {
  sonarUpdateDuration = millis() - lastSonarPing;
  if (sonarUpdateDuration >= pingSpeed) {
    lastSonarPing = millis();

    if (cycleCompleted[currentSonar] == false) {
      //Serial.print("Echo Timeout #");  Serial.println(currentSonar);
      echoDuration[currentSonar] = 11524.72;
      detachInterrupt(digitalPinToInterrupt(sonarPin[currentSonar]));
    } else finalDistance[currentSonar] = echoDuration[currentSonar] / 10000.00 * (speedOfSound + currentTemp * tempModifier + currentHumidity * humidityModifier) / 2;

    if (currentSonar == 5) currentSonar = 0;
    else currentSonar++;

    cycleCompleted[currentSonar] = false;
    //Serial.print("Triggering #");  Serial.println(currentSonar);

    pinMode(sonarPin[currentSonar], OUTPUT);
    digitalWrite(sonarPin[currentSonar], LOW);
    delayMicroseconds(2);
    digitalWrite(sonarPin[currentSonar], HIGH);
    delayMicroseconds(10);
    digitalWrite(sonarPin[currentSonar], LOW);
    pinMode(sonarPin[currentSonar], INPUT);
    attachInterrupt(digitalPinToInterrupt(sonarPin[currentSonar]), echoISR, CHANGE);  //When sensor changes state, execute echoISR (2 times per echo)
  }
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
      */

      desiredRate[ji] = P + I + D;
      desiredRate[ji] = constrain(desiredRate[ji], integralWindupLimitMin, integralWindupLimitMax);
      desiredRatePWM[ji] = map(desiredRate[ji],constAngleP*(-1000),constAngleP*1000,1000,2000);
    }

    inputRate[0] = map(gyroX, -75, 75, 1000, 2000);  // Get Roll
    inputRate[1] = map(gyroY, -75, 75, 1000, 2000);  // Get Pitch
    inputRate[2] = map(gyroZ, -75, 75, 1000, 2000);  // Get Yaw

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

    motorInput[0] = throttleInput - PIDoutput[0] + PIDoutput[1] - PIDoutput[2];
    motorInput[1] = throttleInput - PIDoutput[0] - PIDoutput[1] + PIDoutput[2];  // TBD Review these before flying (EP 11)
    motorInput[2] = throttleInput + PIDoutput[0] - PIDoutput[1] - PIDoutput[2];
    motorInput[3] = throttleInput + PIDoutput[0] + PIDoutput[1] + PIDoutput[2];

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

  gyroX = g.gyro.x - gyroCalibrationX;  //
  gyroY = g.gyro.y - gyroCalibrationY;  // Acceleration in °/s
  gyroZ = g.gyro.z - gyroCalibrationZ;  //

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

    kalmanAnglePWM[ji] = map(kalmanAngle[ji], -50, 50, 1000, 2000);
  }

  verticalAcceleration = -sin(kalmanAngle[1]*3.142/180) * accelerometerX;
  verticalAcceleration += cos(kalmanAngle[1]*3.142/180) * sin(kalmanAngle[0]*3.142/180)*accelerometerY; //TBD See if Im using correct variables
  verticalAcceleration += cos(kalmanAngle[1]*3.142/180) * cos(kalmanAngle[0]*3.142/180)*accelerometerZ;

  verticalAcceleration = (verticalAcceleration-1)*9.81*100; // Convert to cm/s^2 // TBD
  verticalSpeed = verticalSpeed + verticalAcceleration * motorUpdateDurationSeconds;

  //Serial.print(kalmanAngle[0]);  Serial.print("\t");
  //Serial.print(kalmanUncertainty[0]);  Serial.print("\t");
  //Serial.print(kalmanAngle[1]);  Serial.print("\t");
  //Serial.println(kalmanUncertainty[1]);
}

void getAdafruit() {
  
}

void kalman2D(void) {
  Acc = {verticalAcceleration};
  S = F*S + G*Acc;
  P = F*P *~ F + Q;
  L = H*P *~ H + R;
  K = P *~ H*Invert(L);
  M = {currentAltitude};
  S = S + K*(M - H*S);
  altitudeKalman = S(0,0);
  verticalVelocityKalman = S(1,0);
  P = (I-K*H)*P;
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

void echoISR() {
  if (digitalRead(sonarPin[currentSonar]) == HIGH) echoStart = micros();
  else {
    echoDuration[currentSonar] = micros() - echoStart;
    //Serial.print("Received Ping #");  Serial.println(currentSonar);
    cycleCompleted[currentSonar] = true;
    detachInterrupt(digitalPinToInterrupt(sonarPin[currentSonar]));
  }
}

// Unused

/*

void rateModeFlightControllerLoop() {
  motorUpdateDuration = micros() - lastMotorUpdate;
  if (motorUpdateDuration >= motorUpdateSpeed && PIDdisabled == false) {
    lastMotorUpdate = micros();
    motorUpdateDurationSeconds = motorUpdateDuration / 1000000.0;

    getGyro();  // Get gyroX,Y,Z and accelerometerX,Y,Z

    // map(acceleration,-75,75,1000,2000); Drone angle to μs OR 20/3*acceleration+1500;
    inputRate[0] = map(gyroX, -75, -75, 1000, 2000);  // Get Roll
    inputRate[1] = map(gyroY, -75, -75, 1000, 2000);  // Get Pitch
    inputRate[2] = map(gyroZ, -75, -75, 1000, 2000);  // Get Yaw

    //throttleInput = controllerInstructions.throttleInput;
    //desiredRate[0] = controllerInstructions.desiredRate[0]; // TBD
    //desiredRate[1] = controllerInstructions.desiredRate[1]; // In theory, if these 3 are equal they eliminate each other's forces resulting in a stabilised system, except the needed throttle to go up or down
    //desiredRate[2] = controllerInstructions.desiredRate[2]; //

    throttleInput = 0;
    desiredRate[0] = 1500;  //
    desiredRate[1] = 1500;  // In theory, if these 3 are equal they eliminate each other's forces resulting in a stabilised system, except the needed throttle to go up or down
    desiredRate[2] = 1500;  //

    for (uint8_t j = 0; j < 3; j++) {  // For Roll, Pitch, Yaw
      currentError[j] = desiredRate[j] - inputRate[j];

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

    motorInput[0] = throttleInput - PIDoutput[0] - PIDoutput[1] - PIDoutput[2];
    motorInput[1] = throttleInput + PIDoutput[0] - PIDoutput[1] + PIDoutput[2];  // TBD Review these before flying (EP 11)
    motorInput[2] = throttleInput + PIDoutput[0] + PIDoutput[1] - PIDoutput[2];
    motorInput[3] = throttleInput - PIDoutput[0] + PIDoutput[1] + PIDoutput[2];

    for (uint8_t i = 0; i < 4; i++) {
      motorInput[i] = constrain(motorInput[i], minThrottle, 2000);
      servoMotor[i].write(motorInput[i]);
    }
  }
}

*/