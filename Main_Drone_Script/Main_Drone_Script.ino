#include <NewPing.h>
//#define SONAR_NUM 6      // Number of sensors.
#define SONAR_NUM 4      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping. Maximum sensor distance is rated at 400cm.
#define TRIGGER_WIDTH 12 // Microseconds (uS) notch to trigger sensor to start ping. Sensor specs state notch should be 10uS, defaults to 12uS for out of spec sensors. Default=12

const float SpeedOfSound = 331.3;      // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float TempModifier = 0.606;      // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float HumidityModifier = 1.26;   // https://sengpielaudio.com/calculator-airpressure.htm
float CurrentTemp;                   // Celcius
float CurrentHumidity;               // % Humidity
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
  CurrentTemp = 20;      // TBD
  CurrentHumidity = 0.2; // TBD 
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