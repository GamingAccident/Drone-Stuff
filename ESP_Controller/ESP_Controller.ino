#include <esp_now.h>
#include <WiFi.h>

const int emergencyShutdownPin = 39;     // Button // TBD
const int throttleInputPin = 36;         // Button // TBD
const int inputRatePins[3] = {34,35,15}; // Potensiometers // TBD

int emergencyShutdownState = 0;
int throttleInputState = 0;

// TBD all other sensor data
float gyroX;
float gyroY;
float gyroZ;
float accelerometerX;
float accelerometerY;
float accelerometerZ;

uint8_t droneMAC[] = {0xac,0x15,0x18,0x9e,0x19,0xd4}; // MAC adress of the drone to allow for ESPnow connection

struct dataOut { // Packet to send to the drone
  bool emergencyShutdown;
  int throttleInput;
  int inputRate[3];
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

esp_now_peer_info_t peerInfo;

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
 
void setup() {
  Serial.begin(115200);

  pinMode(emergencyShutdownPin, INPUT);
  pinMode(throttleInputPin, INPUT);
  for (uint8_t j = 0; j < 3; j++) pinMode(inputRatePins[j], INPUT);

  WiFi.mode(WIFI_STA); // Set device as a Wi-Fi Station

  if (esp_now_init() != ESP_OK) { // Init ESP-NOW
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // Get the status of Transmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  memcpy(peerInfo.peer_addr, droneMAC, 6); // Register peer
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK){ // Add peer
    Serial.println("Failed to add peer");
    return;
  }

  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv)); // Register for a callback function that will be called when data is received
}
 
void loop() {

  // Check which buttons and knobs are pressed and turned
  emergencyShutdownState = digitalRead(emergencyShutdownPin);
  if (emergencyShutdownState == HIGH) controllerInstructions.emergencyShutdown = true;

  throttleInputState = digitalRead(throttleInputPin);
  if (throttleInputState == HIGH) controllerInstructions.throttleInput += 10; // TBD Do something else with this
  
  for (uint8_t j=0; j<3; j++) controllerInstructions.inputRate[j] = map(analogRead(inputRatePins[j]),0,4095,1000,2000); // TBD Check if this is the range with the new ESP

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(droneMAC, (uint8_t *) &controllerInstructions, sizeof(controllerInstructions));

  //TBD Add all the sensors
  //Serial.print(gyroX);  Serial.print("\t");
  //Serial.print(gyroY);  Serial.print("\t");
  //Serial.print(gyroZ);  Serial.print("\t");
  //Serial.print(accelerometerX);  Serial.print("\t");
  //Serial.print(accelerometerY);  Serial.print("\t");
  //Serial.print(accelerometerZ);  Serial.print("\t");  
  //Serial.println();

  delay(50);
}