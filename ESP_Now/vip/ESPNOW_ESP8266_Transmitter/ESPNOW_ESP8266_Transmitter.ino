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
#include <Arduino.h>

// REPLACE WITH RECEIVER MAC Address
uint8_t broadcastAddress[] = { 0x58, 0xBF, 0x25, 0x4C, 0x0F, 0x80 };  // 58:BF:25:4C:0F:80

#define CHANNEL 1
#define passkey NULL
#define passkey_len 0
#define RCool_pin 3
#define RHeat_pin 1

#define LINE_BUF_SIZE 32

char lineBuf[LINE_BUF_SIZE];
size_t idx = 0;
uint8_t isr_vect=0; 
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;  // send readings timer
uint16_t msg_cnt = 0;
char char_serial;

// Structure example to send data
// Must match the receiver structure
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


// Create a struct_message called myData
struct_message myData;


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
//  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
//  Serial.print("Last Packet Send Status: ");
//  Serial.println( sendStatus == 0 ? "Delivery Success" : "Delivery Fail");
  msg_cnt++;
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

//variables to keep track of the timing of recent interrupts
unsigned long button_time = 0;  
unsigned long last_button_time = 0; 

void ICACHE_RAM_ATTR isr()  {
  button_time = millis();
  if (button_time - last_button_time > 250) {
     isr_vect=1;
     last_button_time = button_time;
  }
}


void setup() {
  // Init Serial Monitor
//  Serial.begin(74880); // 74880 // 115200
//  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  
  // Set global transmiter values 
  {
  myData.NSPanel_ID = 0x5A31;		// NSPanel Zone ID Z1 5A31 
  strcpy(myData.NSPanel_Name, "NSPanel SmartZone ");	// NSPanel Name
  myData.sensor_type = 0x524C; // RL - relay (0x524C), TH - temperature & humidity (0x5448), PW - power meter (0x5057).
  myData.var1;
  strcpy(myData.units1, "RC"); // Relay Cool
  myData.var2;
  strcpy(myData.units2, "RH"); // Relay Heat
  myData.var3;
  strcpy(myData.units3, "NA"); // NA
  myData.raw1;
  myData.raw2;
  myData.raw3; 
  }
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
//  Serial.println("ESP-8266 Transmiter initialization...");
  
/******************************************/  
  //pinMode(RCool_pin, INPUT_PULLUP);
  //pinMode(RHeat_pin, INPUT_PULLUP);
  pinMode(RCool_pin, INPUT_PULLUP);
  pinMode(RHeat_pin, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(RCool_pin), isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RHeat_pin), isr, CHANGE);

/******************************************/


  // configure device AP mode
  //configDeviceAP();  
  
   // This is the mac address of the Master in Station Mode
//  Serial.print("AP MAC: "); Serial.println(WiFi.macAddress());
  if (esp_now_init() == ERR_OK) {}//{ Serial.println("ESP-Now Init Success");  }
  else {
//    Serial.println("Error initializing ESP-NOW");
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
//  Serial.println(esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, CHANNEL, passkey, passkey_len) == 0 ? "Peer add with succes" : "Failed to add peer");
}


void loop() {
  if (isr_vect == 1){
  isr_vect =0;
   myData.var1 = (digitalRead(RCool_pin) == HIGH ? 'Y' : '0' );
   myData.var2 = (digitalRead(RHeat_pin) == HIGH ? 'Y' : '0' );
//   myData.message_cnt = msg_cnt;
   sendData();
   delay(50);
 }
  // Wait for Enter
  // Read all available bytes
  /*
  while (Serial.available() > 0) {
    int ch = Serial.read();

    // End-of-line: Enter can be CR, LF, or CRLF/LFCR
    if (ch == '\r' || ch == '\n') {
      // If CR is followed by LF, consume it (and vice versa)
      if ((ch == '\r' && Serial.peek() == '\n') ||
          (ch == '\n' && Serial.peek() == '\r')) {
        Serial.read(); // swallow paired line ending
      }

      // Terminate string and echo back
      lineBuf[idx] = '\0';
      myData.NSPanel_Name[idx] =  '\0'; 
    //myData.NSPanel_Name = lineBuf;
                             
    myData.Relay_Cool = (digitalRead(RCool_pin) == HIGH ? 'Y' : '0' );
    myData.Relay_Heat = (digitalRead(RHeat_pin) == HIGH ? 'Y' : '0' );
	  myData.message_cnt = msg_cnt;
    sendData();
    //msg_cnt++;
    //lastTime = millis();
    //delay(5000);
      Serial.println();
      Serial.print("Echo: ");
      Serial.println(lineBuf);
      // Reset buffer for next line
      idx = 0;
      continue;
    }
    // Handle backspace (8) or delete (127)
    if (ch == 8 || ch == 127) {
      if (idx > 0) {
        idx--;
        // Optional terminal-friendly backspace echo:
        Serial.print("\b \b");
      }
      continue;
    }
    // Accept printable ASCII
    if (ch >= 32 && ch <= 126) {
      if (idx < LINE_BUF_SIZE - 1) {
        lineBuf[idx] = (char)ch;
        myData.NSPanel_Name[idx] =  (char)ch;
        idx++;
        // Optional live echo of typed character:
        Serial.write((char)ch);
      } else {
        // Buffer full: you can notify or ignore extra chars
        // Here we ignore extras until Enter is pressed
      }
    }
  }
 

  // Keep the watchdog happy during idle waits
  yield();  //*/
delay(100);
}