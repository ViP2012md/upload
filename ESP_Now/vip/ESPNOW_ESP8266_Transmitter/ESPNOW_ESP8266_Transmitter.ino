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
#define RCool_pin 0
#define RHeat_pin 2

#define LINE_BUF_SIZE 128

char lineBuf[LINE_BUF_SIZE];
size_t idx = 0;

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

void isr_RCool() {
  Serial.println("GPIO0 changed!");
}

void isr_RHeat() {
  Serial.println("GPIO2 changed!");
}

void isrGPIO0() {
  Serial.println("GPIO0 changed!");
}

void isrGPIO2() {
  Serial.println("GPIO2 changed!");
}


void setup() {
  // Init Serial Monitor
  Serial.begin(74880); // 74880 // 115200
  Serial.println();
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  Serial.println("ESP-8266 Transmiter initialization...");
  
/******************************************/  
  //pinMode(RCool_pin, INPUT_PULLUP);
  //pinMode(RHeat_pin, INPUT_PULLUP);
  pinMode(0, INPUT_PULLUP);
  //pinMode(2, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(0), isrGPIO0, CHANGE);
 // attachInterrupt(digitalPinToInterrupt(2), isrGPIO2, CHANGE);

/******************************************/


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
  // Wait for Enter
  // Read all available bytes
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
        lineBuf[idx++] = (char)ch;
        // Optional live echo of typed character:
        Serial.write((char)ch);
      } else {
        // Buffer full: you can notify or ignore extra chars
        // Here we ignore extras until Enter is pressed
      }
    }
  }

  // Keep the watchdog happy during idle waits
  yield();
delay(100);

}
 /* 
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
*/
//}

