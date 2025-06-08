#include <esp_now.h>
#include <WiFi.h>

//--- VAR DESCRIPTIONS ---//
// ESP-Now

uint8_t droneMAC[] = {0xac,0x15,0x18,0x9e,0x19,0xd4}; // MAC adress of the drone to allow for ESPnow connection
esp_now_peer_info_t peerInfo;

struct dataOut { // Packet to send to the drone
  bool emergencyShutdown;
  bool shutdown;
  int movementCommand[4]; // Thrust, Roll, Pitch, Yaw
} controllerInstructions;

struct dataIn { // Packet sent to the controller
  // TBD all other sensor data
  float gyroX;
  float gyroY;
  float gyroZ;
  float accelerometerX;
  float accelerometerY;
  float accelerometerZ;
} controllerData;

// Buttons

const int emergencyShutdownPin = 0; // TBD
const int shutdownPin = 0;          // TBD

const int yawControlPin[2] = {0,0}; // Left and Right // TBD

// Potensiometers

const int thrustPin = 0; // TBD

// Joystick

const int joystickPin[2] = {0,0}; // VRX and VRY pin // TBD

// Saved Data for Serial Monitor // TBD all other sensor data

float gyroX;
float gyroY;
float gyroZ;
float accelerometerX;
float accelerometerY;
float accelerometerZ;

//--- PROGRAM START ---//

void setup() {
  Serial.begin(115200);

  pinMode(emergencyShutdownPin, INPUT);                                //
  pinMode(shutdownPin, INPUT);                                         // Initialisation of Buttons
  for (uint8_t ji = 0; ji < 2; ji++) pinMode(yawControlPin[ji], INPUT); //

  controllerInstructions.movementCommand[3] = 1500;

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
}
 
void loop() {

  // Check which buttons and knobs are pressed and turned
  if (digitalRead(emergencyShutdownPin) == HIGH) controllerInstructions.emergencyShutdown = true;
  if (digitalRead(shutdownPin) == HIGH) controllerInstructions.shutdown = true;

  controllerInstructions.movementCommand[0] = map(analogRead(thrustPin),0,4095,0,1000); // TBD Check if this is the range with the new ESP

  controllerInstructions.movementCommand[1] = map(analogRead(joystickPin[0]),0,4095,1000,2000); // TBD Check if this is the range with the new ESP // https://www.diyengineers.com/2023/05/27/ultimate-guide-to-using-a-joystick-with-arduino-step-by-step-tutorial/
  controllerInstructions.movementCommand[2] = map(analogRead(joystickPin[1]),0,4095,1000,2000); // TBD Check if this is the range with the new ESP // TBD Needs calibration
  
  if (digitalRead(yawControlPin[0]) == HIGH && digitalRead(yawControlPin[1]) == HIGH) {
    if (controllerInstructions.movementCommand[3] > 1500) controllerInstructions.movementCommand[3] -= 10;
    if (controllerInstructions.movementCommand[3] < 1500) controllerInstructions.movementCommand[3] += 10;
  }
  else if (digitalRead(yawControlPin[0]) == HIGH) controllerInstructions.movementCommand[3] += 10;
  else if (digitalRead(yawControlPin[1]) == HIGH) controllerInstructions.movementCommand[3] -= 10;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(droneMAC, (uint8_t *) &controllerInstructions, sizeof(controllerInstructions));

  //Serial.print(movementCommand[0]);  Serial.print("\t");
  //Serial.print(movementCommand[1]);  Serial.print("\t");
  //Serial.print(movementCommand[2]);  Serial.print("\t");
  //Serial.print(movementCommand[3]);  Serial.print("\t");
  //Serial.print(emergencyShutdown);  Serial.print("\t");
  //Serial.println(shutdown);

  delay(20);
}

//--- FUNCTIONS ---//

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) { // Callback when data is sent
  if (status != 0){
    Serial.println("Delivery Failed");
  }
}

void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) { // Callback when data is received
  Serial.println("Data Received");
  memcpy(&controllerData, incomingData, sizeof(controllerData));
  gyroX = controllerData.gyroX;
  gyroY = controllerData.gyroY;
  gyroZ = controllerData.gyroZ;
  accelerometerX = controllerData.accelerometerX;
  accelerometerY = controllerData.accelerometerY;
  accelerometerZ = controllerData.accelerometerZ;
}