/* ESPNOW_ESP8266_Receiver
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp8266-nodemcu-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <ESP8266WiFi.h>
#include <espnow.h>

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  uint16_t message_cnt;
  char NSPanel_Name[16];
  uint8_t NSPanel_Zone;
  char Climate_Mode[4];
  float Current_temp;
  float Target_temp_Cool;
  float Target_temp_Heat;
  bool Relay_Cool;
  bool Relay_Heat;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&myData, incomingData, sizeof(myData));
  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off by making the voltage HIGH
  Serial.print("Bytes received: ");
  Serial.println(len);
   //Serial.print("Char: ");
  Serial.print(myData.NSPanel_Name);
  //Serial.print("Int: ");
  Serial.println(myData.NSPanel_Zone);
  Serial.print("Climate mode: ");
  Serial.println(myData.Climate_Mode);
  Serial.print("Current temperature: ");
  Serial.println(myData.Current_temp);
  Serial.print("Target Cool temperature: ");
  Serial.println(myData.Target_temp_Cool);
  Serial.print("Target Heat temperature: ");
  Serial.println(myData.Target_temp_Heat);
  Serial.print("Relay Cool: ");
  Serial.println(myData.Relay_Cool);
  Serial.print("Relay Heat: ");
  Serial.println(myData.Relay_Heat);
  Serial.print("message count: ");
  Serial.println(myData.message_cnt);
  Serial.println();
  //delay(100);
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED on (Note that LOW is the voltage level
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(74880);
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  Serial.println("ESP-8266 Raceiver initialization...");
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
  
}
