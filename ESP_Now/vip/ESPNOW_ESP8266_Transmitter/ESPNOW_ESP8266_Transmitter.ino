/* ESPNOW_ESP8266_Transmitter.ino
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-esp8266-nodemcu-arduino-ide/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*/

#include <ESP8266WiFi.h>
#include <espnow.h>

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = { 0x58, 0xBF, 0x25, 0x4C, 0x0F, 0x80 };  // 58:BF:25:4C:0F:80

#define CHANNEL 1
#define passkey NULL
#define passkey_len 0

// Structure example to send data
// Must match the receiver structure
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

unsigned long lastTime = 0;
unsigned long timerDelay = 5000;  // send readings timer
uint16_t msg_cnt = 0;
char char_serial;

void sendData() {
    // Send message via ESP-NOW
    digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off by making the voltage HIGH
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    delay(50);
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED on (Note that LOW is the voltage level
}

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println( sendStatus == 0 ? "Delivery Success" : "Delivery Fail");
}

/* config AP SSID
void configDeviceAP() {
  String Prefix = "Slave:";
  String Mac = WiFi.macAddress();
  String SSID = Prefix + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
} //*/


void setup() {
  // Init Serial Monitor
  Serial.begin(115200); // 74880
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  Serial.println("ESP-8266 Transmiter initialization...");
  
  
  // configure device AP mode
  //configDeviceAP();  
  
   // This is the mac address of the Master in Station Mode
  Serial.print("AP MAC: "); Serial.println(WiFi.macAddress());
  if (esp_now_init() == ERR_OK) { Serial.println("ESP-Now Init Success");  }
  else {
    Serial.println("Error initializing ESP-NOW");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
       }
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  //esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, CHANNEL, passkey, passkey_len);
  Serial.println(esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, CHANNEL, passkey, passkey_len) == 0 ? "Peer add with succes" : "Failed to add peer");
}


void loop() {
  if (Serial.available()) {        // If anything comes in Serial (USB),
    char_serial = Serial.read();  // read it and send it out Serial1 (pins 0 & 1)
	Serial.print("Serial read: ");
	Serial.println(char_serial);	
    // Set values to send

    strcpy(myData.NSPanel_Name, "NSPanel Zone-6");
    myData.NSPanel_Zone_ID = 0x5A36;
//    strcpy(myData.Climate_Mode, "Auto");
    myData.Relay_Cool, '0';
    myData.Relay_Heat, 'Y';
	myData.message_cnt = msg_cnt;
    sendData();
    msg_cnt++;
    //lastTime = millis();
    //delay(5000);
  }
}