#include <Wire.h>

//GENERAL
const float SpeedOfSound = 331.3;    // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float TempModifier = 0.606;    // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float HumidityModifier = 1.26; // https://sengpielaudio.com/calculator-airpressure.htm
float CurrentTemp = 25;              // Celcius //TBD
float CurrentHumidity = 0.5;         // % Humidity //TBD
unsigned long currentMillis;         // Saves the current millis()
int total_counter = 0;               // Counts how many loops where completed

//HC-SR04 SUPERSONIC SENSORS
const int SONAR_NUM = 4;                    // Number of sensors //TBD Make it 6
const int pingSpeed = 50;                   // Minimum ms between sensor pings. 50ms would be 20 times a second.
float echoDuration[SONAR_NUM];              // Measured distance in ms (Initiallised in setup)
float FinalDistance[SONAR_NUM];             // Final measured distance in cm
unsigned long lastSonarPing;                // Shows the last time any sensor pinged
int TRIG_ECHO_PIN[SONAR_NUM] = {4,2,12,14}; // Pin numbers for the supersonic sensors //TBD Add UP and DOWN
int currentSonar = 3;                       // Which sonar is awaiting input
bool cycleCompleted[SONAR_NUM];             // Has this sonar received the echo (Initiallised in setup)
unsigned long echoStart;                    // Saves the time the echo starts

//BMP280 SENSOR
float temperature_BMP280; // BMP280
float pressure;           // BMP280
int32_t _t_fine;          // Temperature variable
uint16_t _dig_T1;         // Trimming parameters
int16_t _dig_T2,_dig_T3;  // Trimming parameters
uint16_t _dig_P1;         // Trimming parameters
int16_t _dig_P2,_dig_P3,_dig_P4,_dig_P5,_dig_P6,_dig_P7,_dig_P8,_dig_P9;  // Trimming parameters

//AHT20 SENSOR
float temperature_AHT20;
float humidity;
bool sensor_started = false;
bool sensor_busy = false;
unsigned long measurementDelayAHT20 = 0;

//AHT20+BMP280 SENSORS
float delta = 0;
float minDelta = 10;
float maxDelta = 0;

//Heartbeat
unsigned long HeartbeatMillis = 0;
const long Heartbeatinterval = 5000; //How often to check for Temp and Humidity

//GY-521 GYROSCOPE
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature;            // variables for temperature data
float temperature_GY521;        // temperature data in C

char tmp_str[7]; // Temporary variable used in convert function

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  Wire.begin();

  AHT20_begin();
  BMP280_begin();
  startMeasurementAHT20();

  GY521_initialise();

  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Set inital values to supersonics as if it detects nothing
    echoDuration[i] = 3976.03; 
    cycleCompleted[i] = true;
  }

  delay(5000);
}

void loop() { 
  checkbusyAHT20(); // Included in original code, unsure if needed
  getDataAHT20();   // Included in original code, unsure if needed

  currentMillis = millis();
  if (currentMillis - HeartbeatMillis >= Heartbeatinterval) {
    HeartbeatMillis = currentMillis;
    readTemperatureBMP280(); // Outputs temperature_BMP280
    startMeasurementAHT20(); // Outputs temperature_AHT20 and humidity
    CurrentTemp = (temperature_BMP280+temperature_AHT20)/2;
    CurrentHumidity = humidity/100;
  }
  
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
  
  get_GY521(); // Outputs accelerometer_x,accelerometer_y,accelerometer_z (linear acceleration) and gyro_x,gyro_y,gyro_z (angular velocity)

  printOutputs();
  total_counter ++;
}

void printOutputs() {
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

  Serial.print("ax: "); Serial.print(accelerometer_x); Serial.print("\t");
  Serial.print("ay: "); Serial.print(accelerometer_y); Serial.print("\t");
  Serial.print("az: "); Serial.print(accelerometer_z); Serial.print("\t");
  Serial.print("gx: "); Serial.print(gyro_x); Serial.print("\t");
  Serial.print("gy: "); Serial.print(gyro_y); Serial.print("\t");
  Serial.print("gz: "); Serial.print(gyro_z); Serial.print("\t");

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

void GY521_initialise() {
  Wire.beginTransmission(MPU_ADDR); // Begins a transmission to the I2C slave (GY-521 board)
  Wire.write(0x6B); // PWR_MGMT_1 register
  Wire.write(0);    // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void get_GY521() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H) [MPU-6000 and MPU-6050 Register Map and Descriptions Revision 4.2, p.40]
  Wire.endTransmission(false); // the parameter indicates that the Arduino will send a restart. As a result, the connection is kept active.
  Wire.requestFrom(MPU_ADDR, 7*2, true); // request a total of 7*2=14 registers
  
  // "Wire.read()<<8 | Wire.read();" means two registers are read and stored in the same variable
  accelerometer_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x3B (ACCEL_XOUT_H) and 0x3C (ACCEL_XOUT_L)
  accelerometer_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x3D (ACCEL_YOUT_H) and 0x3E (ACCEL_YOUT_L)
  accelerometer_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x3F (ACCEL_ZOUT_H) and 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read()<<8 | Wire.read(); // reading registers: 0x41 (TEMP_OUT_H) and 0x42 (TEMP_OUT_L)
  temperature_GY521 = temperature/340.00+36.53; // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  gyro_x = Wire.read()<<8 | Wire.read(); // reading registers: 0x43 (GYRO_XOUT_H) and 0x44 (GYRO_XOUT_L)
  gyro_y = Wire.read()<<8 | Wire.read(); // reading registers: 0x45 (GYRO_YOUT_H) and 0x46 (GYRO_YOUT_L)
  gyro_z = Wire.read()<<8 | Wire.read(); // reading registers: 0x47 (GYRO_ZOUT_H) and 0x48 (GYRO_ZOUT_L)
  
  /*
  // print out data
  Serial.print("aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature_GY521);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();
  */
}

/*
https://forum.arduino.cc/t/initializing-an-array-arduino-documentation/357403/2
digitalRead() WILL disable PWM on a PWM pin IF you call digitalRead() on the same pin // WUT?
https://mschoeffler.com/2017/10/05/tutorial-how-to-use-the-gy-521-module-mpu-6050-breakout-board-with-the-arduino-uno/
https://github.com/peff74/ESP_AHT20_BMP280

https://docs.google.com/spreadsheets/d/1yThwy3TfXG3--2bJnWaGcEWp8gEH6xALcd34sE_ik74/edit?usp=sharing
*/

//STUFF I HAVEN'T READ BUT ARE NEEDED ------------------------------------------------------

//AHT20+BMP280 SENSORS
void AHT20_begin() {
  Wire.beginTransmission(0x38);
  Wire.write(0xBE);  // 0xBE --> init register for AHT2x
  Wire.endTransmission();
}

void startMeasurementAHT20() {
  Wire.beginTransmission(0x38);
  Wire.write(0xAC);  // 0xAC --> start measurement
  Wire.write(0x33);  // 0x33 --> not really documented what it does, but it's called MEASUREMENT_CTRL
  Wire.write(0x00);  // 0x00 --> not really documented what it does, but it's called MEASUREMENT_CTRL_NOP
  Wire.endTransmission();
  measurementDelayAHT20 = millis();
  sensor_started = true;
  sensor_busy = true;
}

void checkbusyAHT20() {
  if (millis() < measurementDelayAHT20) {
    measurementDelayAHT20 = millis();
  }

  if (sensor_started && sensor_busy && ((millis() - measurementDelayAHT20 >= 200))) {
    sensor_started = false;
    sensor_busy = false;
  }

  if (sensor_started && sensor_busy && ((millis() - measurementDelayAHT20 >= 80))) {
    Wire.requestFrom(0x38, 1);
    if (Wire.available()) {
      unsigned char c = Wire.read();
      if (!(c & 0x80)) {
        sensor_busy = false;
      }
    }
  }
}

void getDataAHT20() {
  if (sensor_started && !sensor_busy) {
    Wire.requestFrom(0x38, 7);  // Request 7 bytes of data

    unsigned char str[7] = { 0 };
    int index = 0;

    // Fault detection
    unsigned long timeoutMillis = 200;
    unsigned long startMillis = millis();

    while (Wire.available()) {
      str[index] = Wire.read();  // Receive a byte as character

      // Debug message: Output of each byte (binary) with labelling
      /***********************************************************/
      // Serial.print("Byte ");
      // Serial.print(index);
      // Serial.print(": ");
      // for (int i = 7; i >= 0; --i) {
      // 	Serial.print((str[index] >> i) & 1);
      // }
      // Serial.println();
      /***********************************************************/

      index++;

      // Fault detection
      if (millis() - startMillis > timeoutMillis) {
        Serial.println("Timeout while waiting for data from AHT20");
        return;
      }
    }
    if (index == 0 || (str[0] & 0x80)) {
      Serial.println("Failed to get data from AHT20");
      sensor_started = false;
      return;
    }

    // Check CRC
    uint8_t crc = 0xFF;
    for (uint8_t byteIndex = 0; byteIndex < 6; byteIndex++) {
      crc ^= str[byteIndex];
      for (uint8_t bitIndex = 8; bitIndex > 0; --bitIndex) {
        if (crc & 0x80) {
          crc = (crc << 1) ^ 0x31;
        } else {
          crc = (crc << 1);
        }
      }
    }
    if (crc != str[6]) {
      Serial.println("CRC check failed");
      sensor_started = false;
      return;
    }

    // Parse data
    float humi, temp;
    // Extract the raw data for humidity from the bytes
    unsigned long __humi = str[1];  // Byte 1: The first 8 bits of the raw data for humidity
    __humi <<= 8;                   // Move the bits 8 positions to the left
    __humi += str[2];               // Byte 2: Add the next 8 bits
    __humi <<= 4;                   // Move the bits 4 positions to the left
    __humi += str[3] >> 4;          // Byte 3: Add the last 4 bits (shifted to the right)

    // Debug message: Output of the value created after the bit shift (binary)
    /************************************************************************/
    // Serial.print("Humidity (raw): ");
    // for (int i = 19; i >= 0; --i) {
    // 	Serial.print((__humi >> i) & 1);
    // }
    // Serial.println();
    /************************************************************************/

    humi = (float)__humi / 1048576.0;
    humidity = humi * 100.0;

    // Extract the raw data for temperature from the bytes
    unsigned long __temp = str[3] & 0x0f;  // Byte 3: The last 4 bits of the raw data for the temperature
    __temp <<= 8;                          // Move the bits 8 positions to the left
    __temp += str[4];                      // Byte 4: Add the next 8 bits
    __temp <<= 8;                          // Move the bits to the left again by 8 positions
    __temp += str[5];                      // Byte 5: Add the last 8 bits


    // Debug message: Output of the value created after the bit shift (binary)
    /************************************************************************/
    // Serial.print("Temperature (raw): ");
    // for (int i = 19; i >= 0; --i) {
    // 	Serial.print((__temp >> i) & 1);
    // }
    // Serial.println();
    /************************************************************************/

    temp = (float)__temp / 1048576.0 * 200.0 - 50.0;

    temperature_AHT20 = temp;

    sensor_started = false;
  }
}

void BMP280_begin() {
  Wire.beginTransmission(0x77);
  Wire.write(0xD0);  // 0xBE -->  register for chip identification
  Wire.endTransmission();
  Wire.requestFrom(0x77, 1);
  uint8_t chip_ID = Wire.read();
  if (chip_ID == 0x58) {  // 0x58 --> BMP280
    Serial.println("BMP280 found");
  } else {
    Serial.println("Unbekannter Sensor.");
  }

  // Generate soft-reset
  Wire.beginTransmission(0x77);
  Wire.write(0xE0);  // 0xE0 --> Reset register
  Wire.write(0xB6);  // 0xB6 --> Reset value for reset register
  Wire.endTransmission();

  // Wait for copy completion NVM data to image registers
  uint8_t stat_Reg = 1;

  while (stat_Reg == 1) {
    Wire.beginTransmission(0x77);
    Wire.write(0XF3);  // 0XF3 --> Status register
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)0x77, (byte)1);
    stat_Reg = Wire.read();
    Serial.println(stat_Reg);
  }

  // See datasheet 4.2.2 Trimming parameter readout

  // Array for storing the read values
  uint16_t values[12];

  // Addresses of the registers with coefficients Data to be read
  uint8_t registers[] = { 0x88, 0x8A, 0x8C, 0x8E, 0x90, 0x92, 0x94, 0x96, 0x98, 0x9A, 0x9C, 0x9E };

  for (int i = 0; i < 12; i++) {
    Wire.beginTransmission(0x77);
    Wire.write(registers[i]);
    Wire.endTransmission();

    Wire.requestFrom((uint8_t)0x77, (byte)2);
    if (Wire.available() >= 2) {
      values[i] = Wire.read() << 8 | Wire.read();  // compose 16-bit value
    }
  }

  // Reverse the bytes for all 16-bit values (little endian / big endian)
  for (int i = 0; i < 12; i++) {
    values[i] = (values[i] >> 8) | (values[i] << 8);
  }

  // Assigning values to the variables
  _dig_T1 = values[0];
  _dig_T2 = values[1];
  _dig_T3 = values[2];
  _dig_P1 = values[3];
  _dig_P2 = values[4];
  _dig_P3 = values[5];
  _dig_P4 = values[6];
  _dig_P5 = values[7];
  _dig_P6 = values[8];
  _dig_P7 = values[9];
  _dig_P8 = values[10];
  _dig_P9 = values[11];

  // Debug message: Output of each byte (binary) with labelling
  /***********************************************************/
  // Serial.println("Temperature Values:");
  // Serial.print("_dig_T1: ");
  // Serial.println(_dig_T1, HEX);
  // Serial.print("_dig_T2: ");
  // Serial.println(_dig_T2, HEX);
  // Serial.print("_dig_T3: ");
  // Serial.println(_dig_T3, HEX);

  // Serial.println("Pressure Values:");
  // Serial.print("_dig_P1: ");
  // Serial.println(_dig_P1, HEX);
  // Serial.print("_dig_P2: ");
  // Serial.println(_dig_P2, HEX);
  // Serial.print("_dig_P3: ");
  // Serial.println(_dig_P3, HEX);
  // Serial.print("_dig_P4: ");
  // Serial.println(_dig_P4, HEX);
  // Serial.print("_dig_P5: ");
  // Serial.println(_dig_P5, HEX);
  // Serial.print("_dig_P6: ");
  // Serial.println(_dig_P6, HEX);
  // Serial.print("_dig_P7: ");
  // Serial.println(_dig_P7, HEX);
  // Serial.print("_dig_P8: ");
  // Serial.println(_dig_P8, HEX);
  // Serial.print("_dig_P9: ");
  // Serial.println(_dig_P9, HEX);
  /***********************************************************/

  // Set in sleep mode to provide write access to the “config” register
  Wire.beginTransmission(0x77);
  Wire.write(0xF4);  // 0XF3 --> Contol register
  Wire.write(0b00);  // 00   --> sleep mode
  Wire.endTransmission();

  // SAMPLING_NONE = 0b000
  // SAMPLING_X1   = 0b001
  // SAMPLING_X2   = 0b010
  // SAMPLING_X4   = 0b011
  // SAMPLING_X8   = 0b100
  // SAMPLING_X16  = 0b101
  
  // MODE_SLEEP  = 0b00
  // MODE_FORCED = 0b01
  // MODE_NORMAL = 0b11

  Wire.beginTransmission(0x77);
  Wire.write(0xF4);
  uint8_t configValues = ((0b001 << 5) | (0b011 << 2) | 0b11);
  //                       temp           press         mode
  Wire.write(configValues);
  Wire.endTransmission();

  delay(10);

  // Set register 0xF5 “config”  ** See datasheet 5.4.6 for details”
  // STANDBY_MS_0_5  = 0b000
  // STANDBY_MS_10   = 0b110
  // STANDBY_MS_20   = 0b111
  // STANDBY_MS_62_5 = 0b001
  // STANDBY_MS_125  = 0b010
  // STANDBY_MS_250  = 0b011
  // STANDBY_MS_500  = 0b100
  // STANDBY_MS_1000 = 0b101

  // FILTER_OFF = 0b000
  // FILTER_X2 = 0b001
  // FILTER_X4 = 0b010
  // FILTER_X8 = 0b011
  // FILTER_X16 = 0b100
  
  Wire.beginTransmission(0x77);
  Wire.write(0xF5);
  configValues = (0b110 << 5) | (0b100 << 2);
  //              standby        filter
  Wire.write(configValues);
  Wire.endTransmission();

  // Wait for first completed conversion
  delay(100);

  // Debug message: Output    of   CTRL_MEAS &  CONFIG (binary)
  /***********************************************************/
  // readAndDisplayRegister(0x77, 0xF4, "CTRL_MEAS");
  // readAndDisplayRegister(0x77, 0xF5, "CONFIG");
  /***********************************************************/
}

void readTemperatureBMP280() {
  int32_t var1, var2, adc_T;

  // Read temperature registers
  Wire.beginTransmission(0x77);
  Wire.write(0xFA);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)0x77, (byte)3);

  adc_T = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();
  adc_T >>= 4;

  // See datasheet 4.2.3 Compensation formulas
  var1 = ((((adc_T >> 3) - ((int32_t)_dig_T1 << 1))) * ((int32_t)_dig_T2)) >> 11;

  var2 = (((((adc_T >> 4) - ((int32_t)_dig_T1)) * ((adc_T >> 4) - ((int32_t)_dig_T1))) >> 12) * ((int32_t)_dig_T3)) >> 14;

  _t_fine = var1 + var2;

  float T = (((_t_fine * 5) + 128) >> 8);

  temperature_BMP280 = T / 100;
}

void readPressureBMP280() {
  int64_t var1;
  int64_t var2;
  int64_t p;
  int32_t adc_P;

  // Read temperature for t_fine
  readTemperatureBMP280();

  // Read pressure registers
  Wire.beginTransmission(0x77);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)0x77, (byte)3);
  adc_P = (Wire.read() << 16) | (Wire.read() << 8) | Wire.read();

  adc_P >>= 4;

  // See datasheet 4.2.3 Compensation formulas
  var1 = ((int64_t)_t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_dig_P6;
  var2 = var2 + ((var1 * (int64_t)_dig_P5) << 17);
  var2 = var2 + (((int64_t)_dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_dig_P3) >> 8) + ((var1 * (int64_t)_dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)_dig_P1) >> 33;

  if (var1 == 0) {
    return;  // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_dig_P7) << 4);

  pressure = p / 25600;
}

void readAndDisplayRegister(uint8_t deviceAddress, byte registerAddress, const char* registerName) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, (byte)1);
  if (Wire.available()) {
    uint8_t registerValue = Wire.read();

    Serial.print(registerName);
    Serial.print(" (");
    Serial.print(registerAddress, HEX);
    Serial.print(") : 0b");

    for (int i = 7; i >= 0; i--) {
      Serial.print((registerValue & (1 << i)) ? '1' : '0');
    }

    Serial.println();
  } else {
    Serial.println("Fehler beim Lesen des Registers");
  }
}

//GY-521 GYROSCOPE
char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}