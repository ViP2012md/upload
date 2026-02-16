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
#define LINE_BUF_SIZE 32

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  uint16_t NSPanel_ID;		// NSPanel Zone ID Z1 5A31 
  char NSPanel_Name[LINE_BUF_SIZE];	// NSPanel Name
  uint16_t sensor_type; // RL - relay (0x524C), TH - temperature & humidity (0x5448), PW - power meter (0x5057).
  //char Relay_Cool;	// Relay Cool		
  //char Relay_Heat;	// Relay Heat		
  uint32_t var1;
  char units1[2]; // Relay Cool
  uint32_t var2;
  char units2[2]; // Relay Heat
  uint32_t var3;
  char units3[2]; // NA
  uint32_t raw1; 
  uint32_t raw2;
  uint32_t raw3;  
} struct_message;
//  char Climate_Mode[4];	// Climate mode // OFF, HEAT, COOL, AUTO
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
  Serial.print("String: ");
  Serial.print(myData.NSPanel_Name);
  Serial.println();
  Serial.print("Zone ID: ");
  uint32_t ZID_int = myData.NSPanel_ID&0xFF00;
  ZID_int = ZID_int >>8; 
  char ZID_1 = ZID_int;
  char ZID_0 = myData.NSPanel_ID&0xFF;
  Serial.print(myData.NSPanel_ID, HEX); Serial.print(" / "); Serial.print(ZID_1);Serial.println(ZID_0);

  if (myData.sensor_type == 0x524C){
	Serial.println("Sensor type : Relay");
    Serial.print("Relay Cool: ");
    Serial.print(myData.units1); Serial.print(" = "); Serial.println(myData.var1,HEX);
    Serial.print("Relay Heat: ");
    Serial.print(myData.units2); Serial.print(" = "); Serial.println(myData.var2,HEX);
  }
  Serial.print("Raw 1: ");
  Serial.println(myData.raw1);
  Serial.print("Raw 2: ");
  Serial.println(myData.raw2);
  Serial.print("Raw 3: ");
  Serial.println(myData.raw3);
  //delay(100);
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED on (Note that LOW is the voltage level
}
 
void setup() {
  // Initialize Serial Monitor
  Serial.begin(74880); // 74880 // 115200
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
