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


// Structure example to send data
// Must match the receiver structure
/*
typedef struct struct_message {
  char a[32];
  int b;
  float c;
  String d;
  bool e;
} struct_message;
*/
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

unsigned long lastTime = 0;
unsigned long timerDelay = 5000;  // send readings timer
uint16_t msg_cnt = 0;

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0) {
    Serial.println("Delivery success");
 //   digitalWrite(LED_BUILTIN, LOW);  // Turn the LED on (Note that LOW is the voltage level
  } else {
    Serial.println("Delivery fail");
 //   digitalWrite(LED_BUILTIN, LOW);  // Turn the LED on (Note that LOW is the voltage level
  }
//delay(100); 
//digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED on (Note that LOW is the voltage level
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200); // 74880
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  Serial.println("ESP-8266 Transmiter initialization...");
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);
}

void loop() {
 // if ((millis() - lastTime) > timerDelay) {
    // Set values to send
    myData.message_cnt = msg_cnt;
    strcpy(myData.NSPanel_Name, "NSPanel Zone-");
    myData.NSPanel_Zone = 6;
    strcpy(myData.Climate_Mode, "Auto");
    myData.Current_temp = 72.5;
    myData.Target_temp_Cool = 76.0;
    myData.Target_temp_Heat = 65.5;
    myData.Relay_Cool = true;
    myData.Relay_Heat = false;

    // Send message via ESP-NOW
    digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off by making the voltage HIGH
   // delay(100); 
    esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));
    delay(50); 
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED on (Note that LOW is the voltage level
    msg_cnt++;
    //lastTime = millis();
    delay(5000);
 // }
}