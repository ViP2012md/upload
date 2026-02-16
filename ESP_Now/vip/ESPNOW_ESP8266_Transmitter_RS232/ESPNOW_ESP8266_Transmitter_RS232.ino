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

#define debug 1
// REPLACE WITH RECEIVER MAC Address
//uint8_t broadcastAddress[] = { 0x58, 0xBF, 0x25, 0x4C, 0x0F, 0x80 };  // 58:BF:25:4C:0F:80
//uint8_t broadcastAddress[] = { 0xEC, 0x64, 0xC9, 0x90, 0xD1, 0xBC }; // EC:64:C9:90:D1:BC
//uint8_t broadcastAddress[] = { 0x30, 0xC9, 0x22, 0xF2, 0x51, 0xE8 }; // 30:C9:22:F2:51:E8
uint8_t broadcastAddress[] = { 0x1C, 0x69, 0x20, 0x00, 0x3E, 0x60 }; // 1C:69:20:00:3E:60 

#define CHANNEL 1
#define passkey NULL
#define passkey_len 0
#define RCool_pin 3
#define RHeat_pin 1

#define LINE_BUF_SIZE 24
char lineBuf[LINE_BUF_SIZE];

uint8_t idx = 0;
uint8_t buf_len = 0;
uint8_t isr_vect=0; 
unsigned long lastTime = 0;
unsigned long timerDelay = 5000;  // send readings timer
uint16_t msg_cnt = 0;
char char_serial;

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  uint16_t NSPanel_ST;  // ZR - zone relay (0x5A52), AC - AC unit relay (0x4143); TH - temperature & humidity (0x5448), PW - power meter (0x5057)
  uint16_t NSPanel_ID;	// NSPanel Zone ID number 00 - 99
  char NSPanel_Name[LINE_BUF_SIZE];	// NSPanel Name
  char Climate_Mode;	// Climate mode : OFF, HEAT, COOL, Dual		
  char NSPanel_sign[12]; // @Viorel.Para 
} struct_message;

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
  Serial.print("Last Packet Sent to: "); Serial.println(macStr);
  Serial.print("Last Packet Send Status: ");
  Serial.println( sendStatus == 0 ? "Delivery Success" : "Delivery Fail");
  msg_cnt++;

	if (debug != 0 ){  
  Serial.println("Data structure send: ");
  Serial.print("Name: ");
  Serial.println(myData.NSPanel_Name);
  //Serial.println();
  if (myData.NSPanel_ST == 0x5A52){	Serial.println("Sensor type : Zone Relay");
    Serial.print("Zone ID - ");
    char int2char = (myData.NSPanel_ID&0xFF00)>>8;
    Serial.print(int2char);
    int2char = myData.NSPanel_ID&0xFF; 
    Serial.println(int2char);
    Serial.print("Climate Mode: ");
//    Serial.print(myData.units1); Serial.print(" = "); 
	Serial.println((char)myData.Climate_Mode);
  } 
  Serial.print("Sign by: ");
  Serial.println(myData.NSPanel_sign);
  }
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

  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  
  // Init Serial Monitor
  Serial.begin(74880); // 74880 // 115200 
  Serial.println(); Serial.println(); 
  Serial.println("ESP-01S Transmiter by @Viorel.Para");
  Serial.println("Initialization ...");

  // Set global transmiter values 
  {
  myData.NSPanel_ST = 0x5A52;  // ZR - zone relay (0x5A52), AC - AC unit relay (0x4143); TH - temperature & humidity (0x5448), PW - power meter (0x5057)
  myData.NSPanel_ID = 0x3031;  // NSPanel Zone ID number 00 - 99
  strcpy(myData.NSPanel_Name, "NSPanel SmartZone ");  // NSPanel Name
  strcpy(myData.NSPanel_sign, "@Viorel.Para");        // @Viorel.Para 
  }
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
//  Serial.println("ESP-8266 Transmiter initialization...");
  
/******************************************/  
  //pinMode(RCool_pin, INPUT_PULLUP);
  //pinMode(RHeat_pin, INPUT_PULLUP);
//  pinMode(RCool_pin, INPUT_PULLUP);
//  pinMode(RHeat_pin, INPUT_PULLUP);

//  attachInterrupt(digitalPinToInterrupt(RCool_pin), isr, CHANGE);
//  attachInterrupt(digitalPinToInterrupt(RHeat_pin), isr, CHANGE);

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
//  Serial.println(esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, CHANNEL, passkey, passkey_len) == 0 ? "Peer add with succes" : "Failed to add peer");
Serial.println("Imput DATA , format type Z1O, Z2C, Z3H, Z4D ...");

}


void loop() {
  uint8_t serial_EOL = 0;
  uint32_t number = 0;
  uint16_t char2int = 0;
  if (isr_vect == 1){
  isr_vect =0;
//   myData.var1 = (digitalRead(RCool_pin) == HIGH ? 'Y' : '0' );
//   myData.var2 = (digitalRead(RHeat_pin) == HIGH ? 'Y' : '0' );
//   myData.message_cnt = msg_cnt;
   sendData();
   delay(50);
 }
  // Wait for Enter
  // Read all available bytes
  /**/
  idx = 0;
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
      lineBuf[idx] = '#';
      Serial.println();
      Serial.print("Echo: "); 
      buf_len = idx;
      for ( idx = 0; idx<=buf_len; idx++ ) { Serial.print( lineBuf[idx] ); }
      Serial.println();

      // Reset buffer for next line
      serial_EOL = 1;
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
    } /**/
    // Accept printable ASCII
    if (ch >= 32 && ch <= 126) {
      if (idx < LINE_BUF_SIZE - 1) {
        lineBuf[idx] = (char)ch;
        //myData.NSPanel_Name[idx] =  (char)ch;
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
  if (serial_EOL == 1){ 
  Serial.println("Converting DATA ...");
    idx = 0;
   // while (lineBuf[idx] != '#'){  
     if ((lineBuf[idx] == 'Z') | (lineBuf[idx] == 'z')){ 
	   Serial.print("lineBuf - "); Serial.print(lineBuf[idx]); Serial.print(lineBuf[idx+1]); Serial.print(lineBuf[idx+2]);
       /*******************************************************************************/
       //myData.NSPanel_ST = 0x5A52;		// NSPanel Zone ID = 0x3031 
       myData.NSPanel_ST =  0x5A52; 
	   char2int = (('0')<<8)+(int)lineBuf[idx+1]; Serial.print(" char2int - "); Serial.print(char2int,HEX);
       myData.NSPanel_ID = char2int; 
	   Serial.print("; idx - "); Serial.println(idx);
       idx+=2;
     }
     
	 if ( ((lineBuf[idx] == 'O') | (lineBuf[idx] == 'o')) ){ Serial.print("lineBuf - "); Serial.print(lineBuf[idx]); Serial.print(lineBuf[idx+1]);
        myData.Climate_Mode = '0'; Serial.print("; myData.Climate_Mode - "); Serial.println((char)myData.Climate_Mode); Serial.print("; idx - "); Serial.println(idx);
        idx+=1;
     } 
	 if ( ((lineBuf[idx] == 'C') | (lineBuf[idx] == 'c')) ){ Serial.print("lineBuf - "); Serial.print(lineBuf[idx]); Serial.print(lineBuf[idx+1]);
        myData.Climate_Mode = 'C'; Serial.print("; myData.Climate_Mode - "); Serial.println((char)myData.Climate_Mode); Serial.print("; idx - "); Serial.println(idx);
        idx+=1;
     }
     if (  ((lineBuf[idx] == 'H') | (lineBuf[idx] == 'h')) ){ Serial.print("lineBuf - "); Serial.print(lineBuf[idx]); Serial.print(lineBuf[idx+1]);
        myData.Climate_Mode = 'H'; Serial.print("; myData.Climate_Mode - "); Serial.println((char)myData.Climate_Mode);  Serial.print("; idx - "); Serial.println(idx);
        idx+=1;
     }
     if ( ((lineBuf[idx] == 'D') | (lineBuf[idx] == 'd')) ){ Serial.print("lineBuf - "); Serial.print(lineBuf[idx]); Serial.print(lineBuf[idx+1]);
        myData.Climate_Mode = 'D'; Serial.print("; myData.Climate_Mode - "); Serial.println((char)myData.Climate_Mode); Serial.print("; idx - "); Serial.println(idx);
        idx+=1;
     }
    if ( lineBuf[idx] == '?') { Serial.println("Imput DATA , format type  Z1O, Z2C, Z3H, Z4D  ..."); lineBuf[idx]= 0; }
    //}
  // EOL 
  sendData();
  buf_len = idx;
  serial_EOL = 0;
  for ( idx = 0; idx<=buf_len; idx++ ) { lineBuf[idx] == '*'; }
  }
delay(100);
}