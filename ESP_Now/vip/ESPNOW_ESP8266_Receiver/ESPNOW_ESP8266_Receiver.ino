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


#define CHANNEL 1
#define passkey NULL
#define passkey_len 0

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  uint16_t NSPanel_ID;		// NSPanel Zone ID Z1 5A31
  char NSPanel_Name[16];	// NSPanel Name
  char Relay_Cool;			// Relay Cool
  char Relay_Heat;			// Relay Heat
  uint16_t message_cnt;		// Message count
} struct_message;
//  char Climate_Mode[4];	// Climate mode // OFF, HEAT, COOL, AUTO


// Create a struct_message called myData
struct_message myData;

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac_addr, uint8_t *incomingData, uint8_t len) {
  
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Recv from: "); Serial.println(macStr); 
  
  
  memcpy(&myData, incomingData, sizeof(myData));
  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off by making the voltage HIGH
  Serial.print("Bytes received: ");
  Serial.println(len);
   //Serial.print("Char: ");
  Serial.print(myData.NSPanel_Name);
  //Serial.print("Int: ");
  Serial.print("Zone ID: ");
  Serial.println(myData.NSPanel_Zone);
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
  Serial.begin(115200); //
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  WiFi.disconnect();  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  Serial.println("ESP-8266 Raceiver initialization...");
  // This is the mac address of the Master in Station Mode
  Serial.print("STA MAC: "); Serial.println(WiFi.macAddress());
  
  if (esp_now_init() == ERR_OK) { Serial.println("ESP-Now Init Success"); }
  else {
    Serial.println("Error initializing ESP-NOW");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
       }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
	
  // Chill
  
}
