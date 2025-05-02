#include <NewPing.h>
#define SONAR_NUM 6      // Number of sensors.
#define MAX_DISTANCE 200 // Maximum distance (in cm) to ping. Maximum sensor distance is rated at 400cm.
#define TRIGGER_WIDTH 12 // Microseconds (uS) notch to trigger sensor to start ping. Sensor specs state notch should be 10uS, defaults to 12uS for out of spec sensors. Default=12

const float speedOfSound = 331.3;      // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float tempModifier = 0.606;      // https://en.wikipedia.org/wiki/Speed_of_sound#Speed_of_sound_in_ideal_gases_and_air
const float humidityModifier = 1.26;   // https://sengpielaudio.com/calculator-airpressure.htm
float = currentTemp;                   // Celcius
float = currentHumidity;               // % Humidity
float = finalDistance;                 // cm
float = finalSpeed;                    // m/s

int pingSpeed = 40;      // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer; // Holds the next ping time    

NewPing sonar[SONAR_NUM] = {
  NewPing(2,2,MAX_DISTANCE),   // UP
  NewPing(4,4,MAX_DISTANCE),   // DOWN
  NewPing(12,12,MAX_DISTANCE), // FRONT
  NewPing(13,13,MAX_DISTANCE), // RIGHT
  NewPing(14,14,MAX_DISTANCE), // BACK
  NewPing(15,15,MAX_DISTANCE), // LEFT
};

void setup() {
  Serial.begin(115200);  // Open serial monitor at 115200 baud to see ping results.
  pingTimer = millis();  // Start counting
  currentTemp = 20;      // TBD Add sensor readings
  currentHumidity = 0.2; // TBD Add sensor readings
}

void loop() {
  if (millis() >= pingTimer) {   // pingSpeed milliseconds since last ping, do another ping.
    pingTimer += pingSpeed;      // Set the next ping time.
    sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
  }
  // TBD Add rest of code here
}

void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  if (sonar.check_timer()) { // This is how you check to see if the ping was received.
    for (uint8_t i = 0; i < SONAR_NUM; i++) { // Loop through each sensor and display results.
      Serial.print(i);
      Serial.print("= ");
      Serial.print(sonar[i].ping_cm());
      Serial.print(" / ");
      finalSpeed = speedOfSound*(1+currentTemp*tempModifier+currentHumidity*humidityModifier);
      finalDistance = sonar[i].ping()*finalSpeed/200; // 2 way trip and converting from m/s
      Serial.print(finalDistance);
      Serial.print(" cm   ");
    }
    Serial.println();
  }
}

/*
https://bitbucket.org/teckel12/arduino-new-ping/wiki/Home
https://docs.google.com/spreadsheets/d/1yThwy3TfXG3--2bJnWaGcEWp8gEH6xALcd34sE_ik74/edit?usp=sharing
*/