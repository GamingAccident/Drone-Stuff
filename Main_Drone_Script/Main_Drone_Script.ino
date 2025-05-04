#include <Wire.h>

//GENERAL
const float SpeedOfSound = 331.3;    // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float TempModifier = 0.606;    // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float HumidityModifier = 1.26; // https://sengpielaudio.com/calculator-airpressure.htm
float CurrentTemp = 20;              // Celcius //TBD
float CurrentHumidity = 0.2;         // % Humidity //TBD
unsigned long currentMillis;         // Saves the current millis()
unsigned long currentMicros;         // Saves the current micros()
int currentSonar = 0;                // Which sonar is awaiting input

//SUPERSONIC SENSORS
const int SONAR_NUM = 4;        // Number of sensors //TBD Make it 6
const int pingSpeed = 50;       // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
long echoDuration[SONAR_NUM];   // Measured distance in ms
float FinalDistance[SONAR_NUM]; // Final measured distance in cm
unsigned long lastSonarPing;    // Shows the last time a sensor pinged
int TRIG_ECHO_PIN[SONAR_NUM] = {4,2,12,14}; //TBD Add UP and DOWN

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

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
  Wire.begin();
  AHT20_begin();
  BMP280_begin();
  startMeasurementAHT20();
  //TBD Check if I can get temp and humidity before checking supersonics on startup
}

void loop() { 
  checkbusyAHT20(); //Included in original code, unsure if needed
  getDataAHT20();   //Included in original code, unsure if needed

  delay(20);

  currentMillis = millis();
  if (currentMillis - HeartbeatMillis >= Heartbeatinterval) {
    HeartbeatMillis = currentMillis;
    readTemperatureBMP280();
    startMeasurementAHT20();
    CurrentTemp = (temperature_BMP280+temperature_AHT20)/2;
    CurrentHumidity = humidity/100;
    Serial.print("Temperature:");
    Serial.print(CurrentTemp);
    Serial.print("\t");
    Serial.print("Humidity:");
    Serial.print(CurrentHumidity);
    Serial.print("\t");
    Serial.println();
  }
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor.
    currentMillis = millis();
    if (currentMillis - lastSonarPing >= pingSpeed) {
      lastSonarPing = millis();
      currentSonar = i;
      Serial.print("DEBUG == Triggering #");
      Serial.println(currentSonar);
      pinMode(TRIG_ECHO_PIN[i], OUTPUT);
      digitalWrite(TRIG_ECHO_PIN[i], LOW);
      delayMicroseconds(2);
      digitalWrite(TRIG_ECHO_PIN[i], HIGH);
      delayMicroseconds(10);
      digitalWrite(TRIG_ECHO_PIN[i], LOW);
      pinMode(TRIG_ECHO_PIN[i], INPUT);
      attachInterrupt(digitalPinToInterrupt(TRIG_ECHO_PIN[i]), echoISR, CHANGE); //When sensor i receives a signal, execute echoISR
    }
  }
  if (currentSonar == 0) {
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
      FinalDistance[i] = echoDuration[i]/10000.0*(SpeedOfSound+CurrentTemp*TempModifier+CurrentHumidity*HumidityModifier)/2;
      switch (i) {
        case 0: Serial.print("Front:"); break;
        case 1: Serial.print("Right:"); break;
        case 2: Serial.print("Back:"); break;
        case 3: Serial.print("Left:"); break;
        //case 4: Serial.print("Up"); break; //TBD
        //case 5: Serial.print("Down"); break; //TBD
      }
      Serial.print(FinalDistance[i]);
      Serial.print("\t");
    }
    Serial.println();
  }
}

void echoISR() {
  unsigned long echoStart = micros();
  if (digitalRead(TRIG_ECHO_PIN[currentSonar]) == HIGH) {
  }
  else {
    echoDuration[currentSonar] = micros() - echoStart;
    Serial.println("DEBUG == Received Ping");
  }
}

//WORK OF OTHERS ------------------------------------------------------

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

/*
https://github.com/peff74/ESP_AHT20_BMP280
https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
https://docs.google.com/spreadsheets/d/1yThwy3TfXG3--2bJnWaGcEWp8gEH6xALcd34sE_ik74/edit?usp=sharing
*/