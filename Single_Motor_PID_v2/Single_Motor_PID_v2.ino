#include <Wire.h>
#include <ESP32Servo.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>
#include <SPI.h>

// GENERAL
const float SpeedOfSound = 331.3;    // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float TempModifier = 0.606;    // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float HumidityModifier = 1.26; // https://sengpielaudio.com/calculator-airpressure.htm
float CurrentTemp = 25;              // Celcius //TBD
float CurrentHumidity = 0.5;         // % Humidity //TBD
unsigned long currentMillis;         // Saves the current millis()
int total_counter = 0;               // Counts how many loops where completed

// HC-SR04 SUPERSONIC SENSORS
const int SONAR_NUM = 4;                    // Number of sensors //TBD Make it 6
const int pingSpeed = 50;                   // Minimum ms between sensor pings. 50ms would be 20 times a second.
float echoDuration[SONAR_NUM];              // Measured distance in ms (Initiallised in setup)
float FinalDistance[SONAR_NUM];             // Final measured distance in cm
unsigned long lastSonarPing;                // Shows the last time any sensor pinged
int TRIG_ECHO_PIN[SONAR_NUM] = {10,13,5,2}; // Pin numbers for the supersonic sensors //TBD Add UP and DOWN
int currentSonar = 3;                       // Which sonar is awaiting input
bool cycleCompleted[SONAR_NUM];             // Has this sonar received the echo (Initiallised in setup)
unsigned long echoStart;                    // Saves the time the echo starts

// BMP280 SENSOR
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

Adafruit_BMP280 bmp;

// AHT20 SENSOR
Adafruit_AHTX0 aht;

// GY-521 GYROSCOPE
Adafruit_MPU6050 mpu;

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  Wire.begin();

  if (! aht.begin()) {
    Serial.println("Could not find AHT? Check wiring");
  }

  //unsigned status;
  //status = bmp.begin(0x76);
  //if (!status) {
  // Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
  //                   "try a different address!"));
  // Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
  // Serial.print("ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
  // Serial.print("ID of 0x56-0x58 represents a BMP 280,\n");
  // Serial.print("ID of 0x60 represents a BME 280.\n");
  // Serial.print("ID of 0x61 represents a BME 680.\n");
  //}
  //bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
  //                Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
  //                Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
  //                Adafruit_BMP280::FILTER_X16,      /* Filtering. */
  //                Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Set inital values to supersonics as if it detects nothing
    echoDuration[i] = 3976.03; 
    cycleCompleted[i] = true;
  }

  delay(1000);
}

void loop() { 
  sensors_event_t humidity, temp;
  aht.getEvent(&humidity, &temp);// Outputs humidity.relative_humidity and temp.temperature

  CurrentTemp = temp.temperature;
  CurrentHumidity = humidity.relative_humidity;
  
  currentMillis = millis();
  if (currentMillis - lastSonarPing >= pingSpeed) {
    lastSonarPing = millis();
    if (cycleCompleted[currentSonar] == false) {
      echoDuration[currentSonar] = 11524.72; // Set sensor to "max" range
      detachInterrupt(digitalPinToInterrupt(TRIG_ECHO_PIN[currentSonar]));
    }
    if (currentSonar == 3) {
      currentSonar = 0;
    }
    else { currentSonar ++; }
    cycleCompleted[currentSonar] = false;
    pinMode(TRIG_ECHO_PIN[currentSonar], OUTPUT);
    digitalWrite(TRIG_ECHO_PIN[currentSonar], LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_ECHO_PIN[currentSonar], HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_ECHO_PIN[currentSonar], LOW);
    pinMode(TRIG_ECHO_PIN[currentSonar], INPUT);
    attachInterrupt(digitalPinToInterrupt(TRIG_ECHO_PIN[currentSonar]), echoISR, CHANGE); //When sensor changes state, execute echoISR (2 times per echo)
  }

  printOutputs();
  total_counter ++;
}

void printOutputs() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  // Outputs temp.temperature and a.acceleration.x,a.acceleration.y,a.acceleration.z (linear acceleration) and g.gyro.x,g.gyro.y,g.gyro.z (angular velocity)

  Serial.print("#Loop: "); Serial.print(total_counter); Serial.print("\t");
  Serial.print("Time: "); Serial.print(millis()); Serial.print("\t");

  Serial.print("Temp: "); Serial.print(CurrentTemp); Serial.print("\t");
  Serial.print("Humidity: "); Serial.print(CurrentHumidity); Serial.print("\t");

  Serial.print("Front: "); Serial.print(FinalDistance[0]); Serial.print("\t");
  Serial.print("Right: "); Serial.print(FinalDistance[1]); Serial.print("\t");
  Serial.print("Back: "); Serial.print(FinalDistance[2]); Serial.print("\t");
  Serial.print("Left: "); Serial.print(FinalDistance[3]); Serial.print("\t");
  Serial.print("Up: "); Serial.print(FinalDistance[4]); Serial.print("\t");
  Serial.print("Down: "); Serial.print(FinalDistance[5]); Serial.print("\t");

  Serial.print("ax: "); Serial.print(a.acceleration.x); Serial.print("\t");
  Serial.print("ay: "); Serial.print(a.acceleration.y); Serial.print("\t");
  Serial.print("az: "); Serial.print(a.acceleration.z); Serial.print("\t");
  Serial.print("gx: "); Serial.print(g.gyro.x); Serial.print("\t");
  Serial.print("gy: "); Serial.print(g.gyro.y); Serial.print("\t");
  Serial.print("gz: "); Serial.print(g.gyro.z); Serial.print("\t");

  Serial.println();
}

void echoISR() {
  if (digitalRead(TRIG_ECHO_PIN[currentSonar]) == HIGH) {
    echoStart = micros();
  }
  else {
    echoDuration[currentSonar] = micros() - echoStart;
    FinalDistance[currentSonar] = echoDuration[currentSonar]/10000.00*(SpeedOfSound+CurrentTemp*TempModifier+CurrentHumidity*HumidityModifier)/2;
    cycleCompleted[currentSonar] = true;
    detachInterrupt(digitalPinToInterrupt(TRIG_ECHO_PIN[currentSonar]));
  }
}

/*
https://forum.arduino.cc/t/initializing-an-array-arduino-documentation/357403/2
digitalRead() WILL disable PWM on a PWM pin IF you call digitalRead() on the same pin // WUT?
https://mschoeffler.com/2017/10/05/tutorial-how-to-use-the-gy-521-module-mpu-6050-breakout-board-with-the-arduino-uno/
https://github.com/peff74/ESP_AHT20_BMP280

https://www.instructables.com/How-to-Connect-BMP-280-to-ESP32-Get-Pressure-Tempe/
https://www.esp32learning.com/code/aht20-integrated-temperature-and-humidity-sensor-and-esp32-board-example.php

Board manager URL's:
https://raw.githubusercontent.com/dbuezas/lgt8fx/master/package_lgt8fx_index.json

https://docs.google.com/spreadsheets/d/1yThwy3TfXG3--2bJnWaGcEWp8gEH6xALcd34sE_ik74/edit?usp=sharing
*/