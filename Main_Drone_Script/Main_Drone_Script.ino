/*Test
#include <NewPing.h> test

#define PING_PIN  10  // Arduino pin tied to both trigger and echo pins on the ultrasonic sensor.
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(PING_PIN, PING_PIN, MAX_DISTANCE); // NewPing setup of pin and maximum distance.

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void loop() {
  delay(1000);                     // Wait 50ms between pings (about 20 pings/sec). 29ms should be the shortest delay between pings.
  Serial.print("Ping: ");
  Serial.print(sonar.ping_cm()); // Send ping, get distance in cm and print result (0 = outside set distance range)
  Serial.println("cm");
}
*/

#include <NewPing.h>
//#define SONAR_NUM 6      // Number of sensors.
#define SONAR_NUM 4      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping. Maximum sensor distance is rated at 400cm.
#define TRIGGER_WIDTH 12 // Microseconds (uS) notch to trigger sensor to start ping. Sensor specs state notch should be 10uS, defaults to 12uS for out of spec sensors. Default=12

const float SpeedOfSound = 331.3;    // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float TempModifier = 0.606;    // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float HumidityModifier = 1.26; // https://sengpielaudio.com/calculator-airpressure.htm
float CurrentTemp = 20;              // Celcius //TBD
float CurrentHumidity = 0.2;         // % Humidity //TBD
float FinalDistance;                 // cm

NewPing sonar[SONAR_NUM] = {
  //NewPing(2,2,MAX_DISTANCE), // UP
  //NewPing(4,4,MAX_DISTANCE), // DOWN
  NewPing(4,4,MAX_DISTANCE),   // FRONT
  NewPing(2,2,MAX_DISTANCE),   // RIGHT
  NewPing(10,10,MAX_DISTANCE), // BACK
  NewPing(9,9,MAX_DISTANCE),   // LEFT
};

void setup() {
  Serial.begin(115200); // Open serial monitor at 115200 baud to see ping results.
}

void loop() { 
  for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
    delay(30); //29ms should be the shortest delay between pings.
    Serial.print(i);
    Serial.print(" = ");
    Serial.print(sonar[i].ping_cm());
    Serial.print("/");
    FinalDistance = sonar[i].ping()*SpeedOfSound*(1+CurrentTemp*TempModifier+CurrentHumidity*HumidityModifier)/200; // 2 way trip and converting from m/s
    Serial.print(FinalDistance);
    Serial.print(" cm");
  }
  Serial.println();
}

/*
https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
https://docs.google.com/spreadsheets/d/1yThwy3TfXG3--2bJnWaGcEWp8gEH6xALcd34sE_ik74/edit?usp=sharing
*/