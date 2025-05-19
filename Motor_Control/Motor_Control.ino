#include <Servo.h>

Servo bldc_motor; //creates a "servo" object (the ESC and motor)

int speed;
int increment = 5;
int start_point = 0;
int end_point = 180;
int middle_point = 640;
int lowest_point = 1016;
int highest_point = 457;

/*
int motor_KV_rating; 
int motor_weight;
// Brisko gia kathe moter mires oste na sikonei to baros tou
// Tis bazo oles sto idio scale 0-100
// Boroume na ipologisoume osi basi baros moter
*/

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//GY-521 GYROSCOPE
const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature;            // variables for temperature data
float temperature_GY521;        // temperature data in C

char tmp_str[7]; // temporary variable used in convert function

void setup() {
  bldc_motor.attach (9,1000,2000); //attach the motor to pin 9
  Serial.begin(9600);

  //GY521_initialise();

  bldc_motor.write(90); // 0 - 180 -> 0% - 100% provides a "neutral" pulse. The ESC won't start without this.
}

void loop() {
  /*
  for (int i = start_point; i <= end_point; i=i+increment) {
    Serial.print("i = "); Serial.print(i);
    bldc_motor.write (i); //actually write the value to the motor
    delay(1000); // small delay so you can see what's happening

    get_GY521();
  }
  bldc_motor.write (0);
  */
  
  int analogValueDeg = analogRead(A0);
  Serial.print("Degrees:");
  Serial.print(analogValueDeg);
  Serial.print("\t");

  int analogValuePower = analogRead(A1);

  float voltage = map (analogValuePower, 0, 1023, 0, 100);

  voltage = abs(voltage - 100);
  Serial.print("%Power:");
  Serial.print(voltage);
  Serial.print("\t");

  int motor_voltage = map (voltage, 0, 100, 0, 180); //maps the signal pulse to a percentage
  bldc_motor.write (motor_voltage);

  if ( abs(analogValueDeg - middle_point) <= 10 ) {
    Serial.print("Equilibrium");
    Serial.print("\t");
  }
  Serial.println();
}


void potensiometerTest() {
  int analogValue = analogRead(A0);
  // Rescale to potentiometer's voltage (from 0V to 5V):
  float voltage = floatMap(analogValue, 0, 1023, 0, 100);

  // print out the value you read:
  Serial.print("Analog: ");
  Serial.print(analogValue);
  Serial.print(", Voltage: ");
  Serial.println(voltage);
}

void zeroTo100() {
  delay(1000);

  Serial.println("Increase");
  for (int i = 0; i <= 180; i=i+10) {

    int speed_value = map (i, 0, 180, 0, 100); //maps the signal pulse to a percentage

    Serial.print("i = ");
    Serial.print(i);
    Serial.print("degrees -> speed_value = ");
    Serial.print(speed_value);
    Serial.println('%');

    bldc_motor.write (i); //actually write the value to the motor

    delay(100);// small delay so you can see what's happening
  }

  delay(5000);

  Serial.println("Decrease");
  for (int i = 180; i >= 0; i=i-10) {

    int speed_value = map (i, 0, 180, 0, 100); //maps the signal pulse to a percentage

    Serial.print("i = ");
    Serial.print(i);
    Serial.print("degrees -> speed_value = ");
    Serial.print(speed_value);
    Serial.println('%');

    delay(100);// small delay so you can see what's happening
  }

  delay(5000);
  Serial.println("Stop");
  bldc_motor.write (0);
  delay(5000);
}

/*
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
  
  // print out data
  Serial.print(" | aX = "); Serial.print(convert_int16_to_str(accelerometer_x));
  Serial.print(" | aY = "); Serial.print(convert_int16_to_str(accelerometer_y));
  Serial.print(" | aZ = "); Serial.print(convert_int16_to_str(accelerometer_z));
  // the following equation was taken from the documentation [MPU-6000/MPU-6050 Register Map and Description, p.30]
  Serial.print(" | tmp = "); Serial.print(temperature_GY521);
  Serial.print(" | gX = "); Serial.print(convert_int16_to_str(gyro_x));
  Serial.print(" | gY = "); Serial.print(convert_int16_to_str(gyro_y));
  Serial.print(" | gZ = "); Serial.print(convert_int16_to_str(gyro_z));
  Serial.println();
}
*/