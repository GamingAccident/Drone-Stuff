#include <esp_now.h>
#include <WiFi.h>

// TBD Change left and right buttons

//--- VAR DESCRIPTIONS ---//
const int startingDelay = 3000; // Time before loop starts (ms)

// ESP-Now

uint8_t droneMAC[] = {0xac,0x15,0x18,0x9e,0x19,0xd4}; // MAC adress of the drone to allow for ESPnow connection
esp_now_peer_info_t peerInfo;

struct dataOut { // Packet to send to the drone
  bool emergencyShutdown = false;
  bool shutdown = false;
  int movementCommand[4] = {0, 1500, 1500, 1500}; // Thrust, Roll, Pitch, Yaw
} controllerInstructions;

struct dataIn { // Packet sent to the controller
  float kalmanAngle[2] = {0,0}; // Thrust and Roll
  float inputRateYaw = 0;
  float motorInput[4] = {0,0,0,0}; // Starting top right, clockwise
  float randomData[10] = {0,0,0,0,0,0,0,0,0,0};
} controllerData;

// Buttons

const int emergencyShutdownPin = 23; // Pin number of button for emergency shutdown
const int shutdownPin = 22;          // Pin number of button for shutdown

const int yawControlPin[2] = {21,19}; // Left and Right

// Potensiometers

const int thrustPin = 36; // Pin number of potensiometer controlling thrust

// Joystick

const int joystickPin[2] = {39,34};   // X and Y axis joystick pin
const int joystickTests = 1000;       // How many tests before calibrating joystick
float joystickCalibration[2] = {0,0}; // Value of the calibration
int analogJoystickInput[2] = {0,0};   // Roll and Pitch inputs from the joystick

//--- PROGRAM START ---//

void setup() {
  Serial.begin(115200);

  pinMode(emergencyShutdownPin, INPUT_PULLDOWN); // Initialisation of Emergency Shutdown Button
  pinMode(shutdownPin, INPUT_PULLDOWN);          // Initialisation of Shutdown Button
  pinMode(thrustPin, INPUT);                     // Initialisation of Potensiometer (Thrust)
  for (uint8_t ji = 0; ji < 2; ji++) {
    pinMode(yawControlPin[ji], INPUT_PULLDOWN);  // Initialisation of Yaw Buttons
    pinMode(joystickPin[ji], INPUT);             // Initialisation of Joystick (Roll and Pitch)
  }

  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station

  if (esp_now_init() != ESP_OK) { // Initialise ESP-NOW
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent); // Register for a callback function that will be called when data is sent
  
  memcpy(peerInfo.peer_addr, droneMAC, 6); //
  peerInfo.channel = 0;                    // Register peer
  peerInfo.encrypt = false;                //
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) { // Add peer
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv)); // Register for a callback function that will be called when data is received

  for (int calibrationNumber = 0; calibrationNumber<joystickTests; calibrationNumber++) {
    for (uint8_t ji = 0; ji < 2; ji++) joystickCalibration[ji] += analogRead(joystickPin[ji]);
  }
  for (uint8_t ji = 0; ji < 2; ji++) joystickCalibration[ji] /= joystickTests;

  Serial.println("Setup Complete");
  delay(startingDelay);
}
 
void loop() {

  // Check which buttons and knobs are pressed and turned
  if (digitalRead(emergencyShutdownPin) == HIGH) controllerInstructions.emergencyShutdown = true;
  if (digitalRead(shutdownPin) == HIGH) controllerInstructions.shutdown = true;

  controllerInstructions.movementCommand[0] = map(analogRead(thrustPin),0,4095,1180,2000);
  
  analogJoystickInput[0] = analogRead(joystickPin[0])-joystickCalibration[0]+4095/2;
  analogJoystickInput[1] = analogRead(joystickPin[1])-joystickCalibration[1]+4095/2;
  controllerInstructions.movementCommand[1] = map(analogJoystickInput[0],0,4095,1250,1750); // TBD Check if this is the range with the new ESP // https://www.diyengineers.com/2023/05/27/ultimate-guide-to-using-a-joystick-with-arduino-step-by-step-tutorial/
  controllerInstructions.movementCommand[2] = map(analogJoystickInput[1],0,4095,1250,1750); // TBD Check if this is the range with the new ESP // TBD Needs further calibration
  
  if (digitalRead(yawControlPin[0]) == HIGH && digitalRead(yawControlPin[1]) == HIGH) {
    if (controllerInstructions.movementCommand[3] > 1500) controllerInstructions.movementCommand[3] -= 10;
    if (controllerInstructions.movementCommand[3] < 1500) controllerInstructions.movementCommand[3] += 10;
  }
  else if (digitalRead(yawControlPin[0]) == HIGH) controllerInstructions.movementCommand[3] += 10;
  else if (digitalRead(yawControlPin[1]) == HIGH) controllerInstructions.movementCommand[3] -= 10;
  controllerInstructions.movementCommand[3] = constrain(controllerInstructions.movementCommand[3],1000,2000);

  esp_err_t result = esp_now_send(droneMAC, (uint8_t *) &controllerInstructions, sizeof(controllerInstructions)); // Send message via ESP-NOW

  //printDataToSend();

  printDataReceived();

  delay(20);
}

//--- FUNCTIONS ---//

void printDataToSend() {
  Serial.print(controllerInstructions.movementCommand[0]);  Serial.print("\t");
  Serial.print(controllerInstructions.movementCommand[1]);  Serial.print("\t");
  Serial.print(controllerInstructions.movementCommand[2]);  Serial.print("\t");
  Serial.print(controllerInstructions.movementCommand[3]);  Serial.print("\t");
  Serial.print(controllerInstructions.emergencyShutdown);  Serial.print("\t");
  Serial.print(controllerInstructions.shutdown);  Serial.print("\t");
  Serial.print(analogJoystickInput[0]);  Serial.print("\t");
  Serial.println(analogJoystickInput[1]);
}

void printDataReceived() {
  Serial.print(controllerData.kalmanAngle[0]);  Serial.print("\t");
  Serial.print(controllerData.kalmanAngle[1]);  Serial.print("\t");
  Serial.print(controllerData.inputRateYaw);  Serial.print("\t");
  Serial.print(controllerData.motorInput[0]);  Serial.print("\t");
  Serial.print(controllerData.motorInput[1]);  Serial.print("\t");
  Serial.print(controllerData.motorInput[2]);  Serial.print("\t");
  Serial.print(controllerData.motorInput[3]);  Serial.print("\t");  Serial.print("\t");

  Serial.print(controllerData.randomData[0]);  Serial.print("\t");
  Serial.print(controllerData.randomData[1]);  Serial.print("\t");
  Serial.print(controllerData.randomData[2]);  Serial.print("\t");
  Serial.print(controllerData.randomData[3]);  Serial.print("\t");
  Serial.print(controllerData.randomData[4]);  Serial.print("\t");
  Serial.print(controllerData.randomData[5]);  Serial.print("\t");  Serial.print("\t");
  Serial.print(controllerData.randomData[6]);  Serial.print("\t");
  Serial.print(controllerData.randomData[7]);  Serial.print("\t");
  Serial.print(controllerData.randomData[8]);  Serial.print("\t");
  Serial.println(controllerData.randomData[9]);
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { // Callback when data is sent
  if (status != 0){
    Serial.println("Delivery Failed");
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) { // Callback when data is received
  //Serial.println("Data Received");
  memcpy(&controllerData, incomingData, sizeof(controllerData));
}