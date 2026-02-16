/* ESPNOW_ESP32_Receiver
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
 
*/

#include "Arduino.h"
#include <WiFi.h>
#include <esp_now.h>
#include "esp_mac.h"  // required - exposes esp_mac_type_t values
#include "PCF8575.h"


#define LED_BUILTIN 17 // 16 - ERR; 17 - API
#define LED_API 16     // 16 - ERR
#define LED_ERR 17     // 17 - API
#define CHANNEL 1
#define passkey NULL
#define passkey_len 0
#define LINE_BUF_SIZE 32

//***  Zone Port Register  ***//
#define Zone1 0     // Zone-1 Register
#define Zone2 1     // Zone-2 Register
#define Zone3 2     // Zone-3 Register
#define Zone4 3     // Zone-4 Register
#define Zone5 4     // Zone-5 Register
#define Zone6 5     // Zone-6 Register
#define Zone7 6     // Zone-7 Register
#define Zone8 7     // Zone-8 Register
//***  IC1 PCF8575 GPIO Port Expand         ***//
#define IC1_Address 0x20	// A2A1A0 --- 0 0 1'
#define MainZone_Relay  7   //11(P10)	//P0  --- Main Zone Relay Dumper
#define Zone1_Relay 7		// P0  --- Zone-8 Relay Dumper
#define Zone2_Relay 6		// P1  --- Zone-7 Relay Dumper 
#define Zone3_Relay 5		// P2  --- Zone-6 Relay Dumper
#define Zone4_Relay 4		// P3  --- Zone-5 Relay Dumper
#define Zone5_Relay 3		// P4  --- Zone-4 Relay Dumper
#define Zone6_Relay 2		// P5  --- Zone-3 Relay Dumper
#define Zone7_Relay 1		// P6  --- Zone-2 Relay Dumper
#define Zone8_Relay 0		// P7  --- Zone-1 Relay Dumper
#define Relay_Comp  11 	//8 // P10 --- Relay-5 Compressor
#define Relay_Heat1 12	//9 // P11 --- Relay-4 HEAT
#define Relay_Cool2 13	//10// P12 --- Relay-3 COOL-2
#define Relay_Cool1 14	//11// P13 --- Relay-2 COOL-1
#define Relay_FAN   15	//12// P14 --- Relay-1 FAN
#define Relay_Aux1  8	//13// P15 --- NA
#define Relay_Aux2  9	//14// P15 --- NA
#define Relay_Aux3  10	//15// P17 --- NA
//*********************************************//

//***  IC2 PCF8575 GPIO Port Expand         ***//
#define IC2_Address 0x21 // A2A1A0 --- 0 0 0
#define Zone1_Cool 0     // !P0  --- Zone-1 COOL i=0*2	=0
#define Zone1_Heat 1     // !P1  --- Zone-1 HEAT i=0*2+1	=1
#define Zone2_Cool 2     // !P2  --- Zone-2 COOL i=1*2	=2
#define Zone2_Heat 3     // !P3  --- Zone-2 HEAT i=1*2+1	=3
#define Zone3_Cool 4     // !P4  --- Zone-3 COOL i=2*2	=4
#define Zone3_Heat 5     // !P5  --- Zone-3 HEAT i=2*2+1	=5
#define Zone4_Cool 6     // !P6  --- Zone-4 COOL i=3*2	=6
#define Zone4_Heat 7     // !P7  --- Zone-4 HEAT i=3*2+1	=7
#define Zone5_Cool 8     // !P10 --- Zone-5 COOL i=4*2	=8
#define Zone5_Heat 9     // !P11 --- Zone-5 HEAT i=4*2+1	=9
#define Zone6_Cool 10    // !P12 --- Zone-6 COOL i=5*2	=10
#define Zone6_Heat 11    // !P13 --- Zone-6 HEAT i=5*2+1	=11
#define Zone7_Cool 12    // !P14 --- Zone-7 COOL i=6*2	=12
#define Zone7_Heat 13    // !P15 --- Zone-7 HEAT i=6*2+1	=13
#define Zone8_Cool 14    // !P15 --- Zone-8 COOL i=7*2	=14
#define Zone8_Heat 15    // !P17 --- Zone-8 HEAT i=7*2+1	=15
//*********************************************//

#define SPI_LED 18     // P0  --- Zone-8 Relay Dumper
#define API_LED 17     // P1  --- Zone-7 Relay Dumper 
#define ERR_LED 16

//***  Main Thermostat Pin Definition  ***//
#define In_FAN 39		// Zone-2 Register
#define In_Cool1 34		// Zone-3 Register
#define In_Cool2 35		// Zone-4 Register
#define In_Heat 32		// Zone-5 Register
#define In_Dehum 33		// Zone-6 Register
#define In_AUX 25		// Zone-7 Register
#define In_24VAC 36		// Zone-1 Register

//***  Main Thermostat Register Definition  ***//
#define MainZone_FAN 0		// Zone-2 Register
#define MainZone_Cool1 1	// Zone-3 Register
#define MainZone_Cool2 2	// Zone-4 Register
#define MainZone_Heat 3		// Zone-5 Register
#define MainZone_Dehum 4	// Zone-6 Register
#define MainZone_AUX 5		// Zone-7 Register
#define MainZone_24VAC 6	// Zone-1 Register

#define LINE_BUF_SIZE 24
char lineBuf[LINE_BUF_SIZE];

//***   Glabal Variable Definition   ***//
bool WirelessDataReceived = false; //true
bool MainZone_set = false;
bool HVAC_set = false; //true

uint8_t HVAC_mode = '0';		// OFF = '0'; Fan = 'F'; Cool1 = 'C';  Heat = 'H';
char MainZone_mode = '0';		// OFF = '0'; Fan = 'F'; Cool1 = 'C';  Heat = 'H';


uint8_t HVAC_nextmode = '0';	// OFF = '0'; Fan = 'F'; Cool1 = 'C';  Heat = 'H';
uint8_t HVAC_Cool_Power = '0';	// OFF = '0'; Cool1 = 'C';  Cool2 = 'K'; Dehum = 'D'; 

uint8_t HVAC_FANmode_next = 0;
uint8_t HVAC_Coolmode_next = 0;
uint8_t HVAC_Heatmode_next = 0;

uint8_t HVAC_FAN_vect = 0;
uint8_t HVAC_Cool_vect = 0;
uint8_t HVAC_Heat_vect = 0;

uint8_t HVAC_delay = 0;

uint8_t FAN_Relay_cnt = 0;
uint8_t Cool_Relay_cnt = 0;
uint8_t Heat_Relay_cnt = 0;

uint8_t Zone_FAN_Register[16] = {0};
uint8_t Zone_Cool_Register[16] = {0};
uint8_t Zone_Heat_Register[16] = {0};
uint8_t Zone_Relay_Register[16] = {0}; 

/// set bit 6 to 1 : a |= (1 << 6);  
/// clear bit 6 to 0 : a &= ~(1 << 6);
/// toggle bit 6 : a ^= (1 << 6);

uint8_t idx = 0, ix = 0, buf_len = 0;
char char_serial; 

bool serial_EOL = false;
uint32_t number = 0;


// Instantiate Wire for generic use at 400kHz
TwoWire I2Cone = TwoWire(0);
// Set pcf8575 i2c comunication with second Wire using 21 22 as SDA SCL
PCF8575 ic1_pcf8575(&I2Cone,IC1_Address);
PCF8575 ic2_pcf8575(&I2Cone,IC2_Address);


// Debounce settings
const unsigned long debounceDelay = 5; // ms

// Shared group: MainZoneISR
volatile bool MainZoneISRTriggered = false;
volatile bool MainZoneRegister[5] = {false};
unsigned long lastInterruptTime = 0;

// Separate: AUX
volatile bool Triggered_AUX = false;
unsigned long lastAuxInterruptTime = 0;

// Separate: 24VAC
volatile bool Triggered_24VAC = false;
unsigned long lastVacInterruptTime = 0;

// Interrupt Service Routines (ISRs)

// ---------------- MainZone ISR -------------
void IRAM_ATTR MainZoneISR() {
  unsigned long currentTime = millis();
  if ((currentTime - lastInterruptTime) > debounceDelay) {
	MainZone_set = true;
	lastInterruptTime = currentTime;
  } 
  
}	

/*
// ---------------- MainZone ISR -------------
void IRAM_ATTR MainZoneISR() {
  unsigned long currentTime = millis();

  if (currentTime - lastInterruptTime[0] > debounceDelay) {
	  if (digitalRead(In_FAN) == HIGH){     
		lastInterruptTime[0] = currentTime;
		MainZoneRegister[0] = true;

		MainZone_set = true;
	  } else {
		lastInterruptTime[0] = currentTime;
		MainZoneRegister[0] = false;

		MainZone_set = true;		  
	  }	
  }
  
  if (currentTime - lastInterruptTime[1] > debounceDelay) {
	  if (digitalRead(In_Cool1) == HIGH){
		lastInterruptTime[1] = currentTime;
		MainZoneRegister[1] = true;

		MainZone_set = true;
	  } else {
		lastInterruptTime[1] = currentTime;
		MainZoneRegister[1] = false;

		MainZone_set = true;		  
	  }	

  }
  
  if (currentTime - lastInterruptTime[2] > debounceDelay) {
	  if (digitalRead(In_Cool2) == HIGH){
		lastInterruptTime[2] = currentTime;
		MainZoneRegister[2] = true;

		MainZone_set = true;
	  } else {
		lastInterruptTime[2] = currentTime;
		MainZoneRegister[2] = false;

		MainZone_set = true;		  
	  }	

  }
  
  if (currentTime - lastInterruptTime[3] > debounceDelay) {
	  if (digitalRead(In_Heat) == HIGH){
		lastInterruptTime[3] = currentTime;
		MainZoneRegister[3] = true;

		MainZone_set = true;
	  } else {
		lastInterruptTime[3] = currentTime;
		MainZoneRegister[3] = false;

		MainZone_set = true;		  
	  }	

  }
  
  if (currentTime - lastInterruptTime[4] > debounceDelay) {
	  if (digitalRead(In_Dehum) == HIGH){
		lastInterruptTime[4] = currentTime;
		MainZoneRegister[4] = true;

		MainZone_set = true;
	  } else {
		lastInterruptTime[4] = currentTime;
		MainZoneRegister[4] = false;

		MainZone_set = true;		  
	  }
  }
}


*/

// ---------------- AUX ISR ----------------
void IRAM_ATTR ISR_AUX() {
  unsigned long currentTime = millis();
  if (currentTime - lastAuxInterruptTime > debounceDelay) {
    Triggered_AUX = true;
    lastAuxInterruptTime = currentTime;
  }
}

// ---------------- 24VAC ISR ----------------
void IRAM_ATTR ISR_24VAC() {
  unsigned long currentTime = millis();
  if (currentTime - lastVacInterruptTime > debounceDelay) {
    Triggered_24VAC = true;
    lastVacInterruptTime = currentTime;
  }
}

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  uint16_t NSPanel_ST;  // ZR - zone relay (0x5A52), AC - AC unit relay (0x4143); TH - temperature & humidity (0x5448), PW - power meter (0x5057)
  uint16_t NSPanel_ID;	// NSPanel Zone ID number 00 - 99
  char NSPanel_Name[LINE_BUF_SIZE];	// NSPanel Name
  char Climate_Mode;	// Climate mode : OFF, HEAT, COOL, Dual			
  char NSPanel_sign[12]; // @Viorel.Para
} struct_message;
// Create a struct_message called myData
struct_message myData;

// Callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED off by making the voltage HIGH
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.println();
  Serial.print("<< ");  Serial.print(len);
  Serial.print(" Bytes received from: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(mac_addr[i], HEX);
    if (i < 5) Serial.print(":");
  }
  Serial.println(" >>");
  WirelessDataReceived = true;
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED on (Note that LOW is the voltage level
}

// LED Port Test function that will be executed when device is initialised
void LEDPortTest() {
 // TESTING OUTPUT 
  // Set OUTPUT LOW
   for(int i=0;i<16;i++) {
    ic1_pcf8575.digitalWrite(i, HIGH);
    ic2_pcf8575.digitalWrite(i, LOW);
  } //*/
  delay(500);
  // Set OUTPUT Hight
   for(int i=0;i<16;i++) {
    ic1_pcf8575.digitalWrite(i, LOW);
    ic2_pcf8575.digitalWrite(i, HIGH);
  } //*/
//  delay(500);
  // Set OUTPUT Hight
/*   for(int i=0;i<16;i++) {
    ic1_pcf8575.digitalWrite(i, HIGH);
	delay(50);
  } 
   for(int i=0;i<16;i++) {
    ic2_pcf8575.digitalWrite(i, LOW);
	delay(50);
  }  
  delay(500); 
  // Set OUTPUT Hight
   for(int i=0;i<16;i++) {
    ic1_pcf8575.digitalWrite(i, LOW);
	delay(50);
  } 
   for(int i=0;i<16;i++) {
    ic2_pcf8575.digitalWrite(i, HIGH);
	delay(50);
  } */ 
 
 
 
}

bool SSH_SerialRecv(){
  uint8_t  idx = 0;
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
      lineBuf[idx] = '#';
    //sendData();
    //msg_cnt++;
    //lastTime = millis();
    //delay(5000);
      digitalWrite(SPI_LED, LOW);
      Serial.println();
      Serial.print("Echo: "); 
      buf_len = idx;
      for ( idx = 0; idx<=buf_len; idx++ ) { Serial.print( lineBuf[idx] ); }
      Serial.println();

      // Reset buffer for next line
      serial_EOL == true;
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
 return serial_EOL;
}

void Convert_SSH_Serial(){
  Serial.print("Converting DATA ... ");
    idx = 0;
 //   while (lineBuf[idx] != '#'){   //////// ic2_pcf8575.digitalWrite(Zone1_Cool, HIGH); ic1_pcf8575.digitalWrite(Zone1_Relay, LOW); 			// Relay and LED is OFF
     if ( ((lineBuf[idx] == 'Z') | (lineBuf[idx] == 'z')) ){ 
	    if (lineBuf[idx+1] == '1'){
		    if ( ((lineBuf[idx+2] == 'C') | (lineBuf[idx] == 'c')) ){
			   ic2_pcf8575.digitalWrite(Zone1_Cool, ( lineBuf[idx+3] == '0' ? HIGH : LOW ) );
			}
		    if ( ((lineBuf[idx+2] == 'H') | (lineBuf[idx] == 'h')) ){
			   ic2_pcf8575.digitalWrite(Zone1_Heat,  ( lineBuf[idx+3] == '0' ? HIGH : LOW ) );
			}		
		    if ( ((lineBuf[idx+2] == 'R') | (lineBuf[idx] == 'r')) ){
			   ic1_pcf8575.digitalWrite(Zone1_Relay,  ( lineBuf[idx+3] == '0' ? LOW : HIGH ) );
			}		
		}
		if (lineBuf[idx+1] == '2') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone2_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone2_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone2_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
		if (lineBuf[idx+1] == '3') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone3_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone3_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone3_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
		if (lineBuf[idx+1] == '4') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone4_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone4_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone4_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
		if (lineBuf[idx+1] == '5') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone5_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone5_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone5_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
		if (lineBuf[idx+1] == '6') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone6_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone6_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone6_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
		if (lineBuf[idx+1] == '7') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone7_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone7_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone7_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
		if (lineBuf[idx+1] == '8') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone8_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone8_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone8_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
	 }
	 
     if ( ((lineBuf[idx] == 'C') | (lineBuf[idx] == 'c')) ){ 
		    if ( lineBuf[idx+1] == '1'){
			   ic1_pcf8575.digitalWrite(Relay_Cool1, ( lineBuf[idx+2] == '0' ? LOW : HIGH) );
			}
		    if ( lineBuf[idx+1] == '2'){
			   ic1_pcf8575.digitalWrite(Relay_Cool2,  ( lineBuf[idx+2] == '0' ? LOW : HIGH) );
			}	
		    if ( lineBuf[idx+1] == 'P'){
			   ic1_pcf8575.digitalWrite(Relay_Comp,  ( lineBuf[idx+2] == '0' ? LOW : HIGH) );
			}				
		}
		
     if ( ((lineBuf[idx] == 'H') | (lineBuf[idx] == 'h')) ){ 
		ic1_pcf8575.digitalWrite(Relay_Heat1, ( lineBuf[idx+1] == '0' ? LOW : HIGH ) );
		}
		
     if ( ((lineBuf[idx] == 'F') | (lineBuf[idx] == 'f')) ){ 
		ic1_pcf8575.digitalWrite(Relay_FAN, ( lineBuf[idx+1] == '0' ? LOW : HIGH ) );
		}
		
     if ( ((lineBuf[idx] == 'A') | (lineBuf[idx] == 'a')) ){ 
		    if ( lineBuf[idx+1] == '1'){
			   ic1_pcf8575.digitalWrite(Relay_Aux1, ( lineBuf[idx+2] == '0' ? LOW : HIGH) );
			}
		    if ( lineBuf[idx+1] == '2'){
			   ic1_pcf8575.digitalWrite(Relay_Aux2,  ( lineBuf[idx+2] == '0' ? LOW : HIGH) );
			}	
		    if ( lineBuf[idx+1] == '3'){
			   ic1_pcf8575.digitalWrite(Relay_Aux3,  ( lineBuf[idx+2] == '0' ? LOW : HIGH) );
			}
			if ( ((lineBuf[idx+1] == 'L') | (lineBuf[idx+1] == 'l')) & ((lineBuf[idx+2] == 'L') | (lineBuf[idx+2] == 'l')) ){
				if ( lineBuf[idx+3] == '0'){
					for(int i=0;i<16;i++) {
						ic1_pcf8575.digitalWrite(i, LOW);
						ic2_pcf8575.digitalWrite(i, HIGH);
					}  
				} else {            
					for(int i=0;i<16;i++) {
						ic1_pcf8575.digitalWrite(i, HIGH);
						ic2_pcf8575.digitalWrite(i, LOW);
					}
				}
			} 
	 }

    if ( lineBuf[idx] == '?'){ 
	   Serial.println("Imput DATA , format type Z1C*,F*,C1*,C2*,H*,CP* ..."); lineBuf[idx]= 0; 
	}
	
  serial_EOL = false;
  for ( idx = 0; idx<8; idx++ ) { lineBuf[idx] == '*'; }
  Serial.println("Done"); digitalWrite(SPI_LED, HIGH);
	
}  // Convert_SSH_Serial();


bool SmartZone_Mode_Update(char ZoneClimate_Mode, uint8_t Zone_Cool, uint8_t Zone_Heat, uint8_t Zone_Relay){
	if ( ZoneClimate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)ZoneClimate_Mode);
		Zone_FAN_Register[Zone_Relay] = 0;
        Zone_Cool_Register[Zone_Relay] = 0;
        Zone_Heat_Register[Zone_Relay] = 0;	
			
		ic2_pcf8575.digitalWrite(Zone_Cool, HIGH); 
		ic2_pcf8575.digitalWrite(Zone_Heat, HIGH);
		
	}  // ZoneClimate_Mode == '0'
		
	if ( ZoneClimate_Mode == 'F' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)ZoneClimate_Mode);
		Zone_FAN_Register[Zone_Relay] = 1;
		Zone_Cool_Register[Zone_Relay] = 0;
        Zone_Heat_Register[Zone_Relay] = 0;	

		ic2_pcf8575.digitalWrite(Zone_Cool, HIGH); 
		ic2_pcf8575.digitalWrite(Zone_Heat, HIGH);		

		Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);

	}  // ZoneClimate_Mode == 'F'	
		
	if ( ZoneClimate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)ZoneClimate_Mode);
		Zone_FAN_Register[Zone_Relay] = 1;			
		Zone_Cool_Register[Zone_Relay] = 1;
        Zone_Heat_Register[Zone_Relay] = 0;			

		ic2_pcf8575.digitalWrite(Zone_Cool, LOW); 
		ic2_pcf8575.digitalWrite(Zone_Heat, HIGH);
			
		HVAC_Coolmode_next = 1;
		Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);

	}  // ZoneClimate_Mode == 'C'
		
	if ( ZoneClimate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)ZoneClimate_Mode);
		Zone_FAN_Register[Zone_Relay] = 1;
		Zone_Cool_Register[Zone_Relay] = 0;
        Zone_Heat_Register[Zone_Relay] = 1;			

		ic2_pcf8575.digitalWrite(Zone_Cool, HIGH); 
		ic2_pcf8575.digitalWrite(Zone_Heat, LOW); 			
			
		HVAC_Heatmode_next = 1;
		Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
	}  // ZoneClimate_Mode == 'H'
		
return true;
}  // SmartZone_Mode_Update





void HVAC_Core_Update(){
	
  if ( HVAC_set == true ){ 
	Serial.print("HVAC Core Update: ");    Serial.println(HVAC_set);
	ix = 0;
	
	FAN_Relay_cnt = 0;
	Cool_Relay_cnt = 0;
	Heat_Relay_cnt = 0;
	
	HVAC_FAN_vect = 0;
	HVAC_Cool_vect = 0;
	HVAC_Heat_vect = 0;

	Serial.print("Zone_FAN_Register:");
    for(idx=0;idx<16;idx++) { 
		HVAC_FAN_vect|=Zone_FAN_Register[idx]; 
		if (Zone_FAN_Register[idx] == true){
			FAN_Relay_cnt++;
		}
		
		Serial.print(" F");    Serial.print(Zone_FAN_Register[idx]);	
    } Serial.println();			
	
	Serial.print("Zone_Cool_Register:");
    for(idx=0;idx<16;idx++) {	
		HVAC_Cool_vect|=Zone_Cool_Register[idx];
		if (Zone_Cool_Register[idx] == true){
			Cool_Relay_cnt++;
		}		
		Serial.print(" C");    Serial.print(Zone_Cool_Register[idx]);
    } Serial.println();	
	
	Serial.print("Zone_Heat_Register:");
    for(idx=0;idx<16;idx++) { 
		HVAC_Heat_vect|=Zone_Heat_Register[idx]; 
		if (Zone_Heat_Register[idx] == true){
			Heat_Relay_cnt++;
		}		
		Serial.print(" H");    Serial.print(Zone_Heat_Register[idx]); 
    } Serial.println();	
		
	Serial.print("#HVAC_FAN_vect: ");    Serial.println(HVAC_FAN_vect);
	Serial.print("#HVAC_Cool_vect: ");    Serial.println(HVAC_Cool_vect);
	Serial.print("#HVAC_Heat_vect: ");    Serial.println(HVAC_Heat_vect);
		
    // Update HVAC Status to OFF or IDLE
	if (( HVAC_FAN_vect == 0 )&( HVAC_Cool_vect == 0 )&( HVAC_Heat_vect == 0 )) { 
		HVAC_mode = '0';
		//HVAC_FANmode_next = 0;
		//HVAC_Coolmode_next = 0;
		//HVAC_Heatmode_next = 0;
		Serial.print("!HVAC_mode: "); Serial.println((char)HVAC_mode);
		Serial.print("!HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
	}
	// Update HVAC Status to FAN mode
	if ( ( HVAC_FAN_vect == 1 )&(( HVAC_Cool_vect == 0 )&( HVAC_Heat_vect == 0 )))  { 
		HVAC_mode = 'F';
		HVAC_FANmode_next = 0;
		Serial.print("%HVAC_mode: "); Serial.println((char)HVAC_mode);
		Serial.print("%HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
	}
	// Update HVAC Status to Cool mode
	if (( HVAC_Cool_vect == 1 )&(( HVAC_mode != 'H' )|( HVAC_Heat_vect == 0 )&( HVAC_Coolmode_next == 1 )))  { 
		HVAC_mode = 'C';
		HVAC_Coolmode_next = '0'; 
		Serial.print("@HVAC_mode: "); Serial.println((char)HVAC_mode);
		Serial.print("@HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
	}
	// Update HVAC Status to Heat mode
	if (( HVAC_Heat_vect == 1 )&(( HVAC_mode != 'C' )|( HVAC_Cool_vect == 0 )&( HVAC_Heatmode_next == 1 ))) { 
		HVAC_mode = 'H'; 
		HVAC_Heatmode_next = '0'; 
		Serial.print("@HVAC_mode: "); Serial.println((char)HVAC_mode);
		Serial.print("@HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
	}

	// Set HVAC Ouput to OFF or IDLE
	if ( HVAC_mode == '0' ){ 
		Serial.print("$HVAC_mode: ");    Serial.println((char)HVAC_mode);

        for(idx=0;idx<16;idx++) {
			Zone_Relay_Register[idx] = 0;		
		}			

	} // HVAC_mode == '0' 
	
	// Set HVAC Ouput to FAN mode
    if ( HVAC_mode == 'F' ){ 
		Serial.print("$HVAC_mode: ");    Serial.println((char)HVAC_mode);
		ix = 0;

        for(idx=0;idx<16;idx++) {
			Zone_Relay_Register[idx] = Zone_FAN_Register[idx];	
		}

		Zone_Relay_Register[Relay_FAN] = 1;		//Relay = 1
		Zone_Relay_Register[Relay_Cool1] = 0;	//Relay = 1
		Zone_Relay_Register[Relay_Cool2] = 0;	//Relay = 0
		Zone_Relay_Register[Relay_Heat1] = 0;	//Relay = 0
		Zone_Relay_Register[Relay_Comp] = 0;	//Relay = 1
		
		if (FAN_Relay_cnt > 3){
			// !!!!!!!!!! NEEDS WORK !!!!!!!!!! //
		}
		
	}  // HVAC_mode == 'F'
	
	// Set HVAC Ouput to Cool mode
    if ( HVAC_mode == 'C' ){ 
		Serial.print("$HVAC_mode: ");    Serial.println((char)HVAC_mode);
		ix = 0;
		Cool_Relay_cnt = 0;

        for(idx=0;idx<16;idx++) {
			Zone_Relay_Register[idx] = Zone_Cool_Register[idx];
		}
		
		// HVAC Cooling Power mode setup
		if (HVAC_Cool_Power == 'C'){ 
			Zone_Relay_Register[Relay_Cool1] = 1;	//Relay = 1
			Zone_Relay_Register[Relay_Cool2] = 0;	//Relay = 0
		}
		if (HVAC_Cool_Power == 'K'){ 
			Zone_Relay_Register[Relay_Cool1] = 1;	//Relay = 1
			Zone_Relay_Register[Relay_Cool2] = 1;	//Relay = 0
			
		}
		if ((HVAC_Cool_Power == 'D')|(HVAC_Cool_Power == '0')){ 
			Zone_Relay_Register[Relay_Cool1] = 0;	//Relay = 1
			Zone_Relay_Register[Relay_Cool2] = 0;	//Relay = 0
		}
		
		Zone_Relay_Register[Relay_FAN] = 1;		//Relay = 1
		Zone_Relay_Register[Relay_Heat1] = 0;	//Relay = 0
		Zone_Relay_Register[Relay_Comp] = 1;	//Relay = 1
		
		if (Cool_Relay_cnt > 3){
			// !!!!!!!!!! NEEDS WORK !!!!!!!!!! //
		}

	}  // HVAC_mode == 'C'
	
	// Set HVAC Ouput to Heat mode
    if ( HVAC_mode == 'H' ){ 
		Serial.print("$HVAC_mode: ");    Serial.println((char)HVAC_mode);
	    ix = 1;
		Heat_Relay_cnt = 0;

        for(idx=0;idx<16;idx++) {
			Zone_Relay_Register[idx] = Zone_Heat_Register[idx];	
		}

		Zone_Relay_Register[Relay_FAN] = 1;		//Relay = 1
		Zone_Relay_Register[Relay_Cool1] = 0;	//Relay = 0
		Zone_Relay_Register[Relay_Cool2] = 0;	//Relay = 0
		Zone_Relay_Register[Relay_Heat1] = 1;	//Relay = 1
		Zone_Relay_Register[Relay_Comp] = 0;	//Relay = 1
		
		if (Heat_Relay_cnt > 3){
			// !!!!!!!!!! NEEDS WORK !!!!!!!!!! //
		}		
	}  // HVAC_mode == 'H'
	
	// Update Relay Ouput Port
	for(idx=0;idx<16;idx++) {
  		if ( Zone_Relay_Register[idx] == 1 ){
			ic1_pcf8575.digitalWrite(idx, HIGH);	//Relay = 1			
			Cool_Relay_cnt++;
		}  else {
			ic1_pcf8575.digitalWrite(idx, LOW);	//Relay = 0
		}
	}
	
  HVAC_set = false; 
  }	  // HVAC_set //  
} 	  // HVAC-Core_Update // 




void setup() {
  // Initialize Serial Monitor
  Serial.begin(921600); // 74880 // 115200
  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  Serial.println();
  
  pinMode(SPI_LED, OUTPUT);
  pinMode(API_LED, OUTPUT);
  pinMode(ERR_LED, OUTPUT);
  
  digitalWrite(SPI_LED, HIGH);
  digitalWrite(API_LED, HIGH);
  digitalWrite(ERR_LED, HIGH);
  
  // Configure pins as inputs with internal pulldown
  pinMode(In_FAN,   INPUT);
  pinMode(In_Cool1, INPUT);
  pinMode(In_Cool2, INPUT);
  pinMode(In_Heat,  INPUT);
  pinMode(In_Dehum, INPUT);
  pinMode(In_AUX,   INPUT);
  pinMode(In_24VAC, INPUT);
  
  // Attach all interrupts to the same ISR
  // Trigger on RISING edge (LOW → HIGH transition)
  attachInterrupt(digitalPinToInterrupt(In_FAN),   MainZoneISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(In_Cool1), MainZoneISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(In_Cool2), MainZoneISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(In_Heat),  MainZoneISR, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(In_Dehum), MainZoneISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(In_AUX),   ISR_AUX,   CHANGE);
  attachInterrupt(digitalPinToInterrupt(In_24VAC), ISR_24VAC, CHANGE);
  
 
  // Initialize I2C port pins as an output 
  TwoWire I2Cone = TwoWire(0);
  // Set pinMode to OUTPUT
  for(int i=0;i<16;i++) {
    ic1_pcf8575.pinMode(i, OUTPUT);
    ic2_pcf8575.pinMode(i, OUTPUT);
  }
  ic1_pcf8575.begin();
  ic2_pcf8575.begin();
  LEDPortTest();
 
  delay(500); 
  // Dissable Wi-Fi Radio  
  WiFi.disconnect();  
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  // Init ESP-NOW
  Serial.println("ESP32 Raceiver initialization...");
  // This is the mac address of the Master in Station Mode

  Serial.print("WiFi STA MAC: ");;
  Serial.println(getInterfaceMacAddress(ESP_MAC_WIFI_STA)); 
  if (esp_now_init() == ESP_OK) { Serial.println("ESP-Now Init Success"); }
  else {
    Serial.println("Error initializing ESP-NOW");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
       }
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  //esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  
  // Register callback function
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}


void loop() {
/***  Relays PinOut Debuging ***************************************************/		
  // Wait for SSH Serial Console
  serial_EOL = SSH_SerialRecv();
  // Keep the watchdog happy during idle waits
  //yield();  /**/ 
  if (serial_EOL == true){ 
  Convert_SSH_Serial();
  } // Convert_SSH_Serial();
//delay(100);
/************************************************************************/	



  if ( WirelessDataReceived == true ){ 
	HVAC_set = false;
    Serial.print("String: ");
    Serial.print(myData.NSPanel_Name);
    //Serial.println();
    if ( myData.NSPanel_ST == 0x5A52 | myData.NSPanel_ST == 0x7A72 ){ 
	//Serial.println("Sensor type : Zone Relay");
//		if (  myData.NSPanel_ID == 0x3031 ){ 
//			Serial.println(" Zone-01 "); 
//			HVAC_set = SmartZone_Mode_Update(myData.Climate_Mode, Zone1_Cool, Zone1_Heat, Zone1_Relay);
//		} // myData.NSPanel_ID 01 //

		if (  myData.NSPanel_ID == 0x3032 ){ 
			Serial.println(" Zone-02 "); 
			HVAC_set = SmartZone_Mode_Update(myData.Climate_Mode, Zone2_Cool, Zone2_Heat, Zone2_Relay);	
		} // myData.NSPanel_ID 02 //

		if (  myData.NSPanel_ID == 0x3033 ){ 
			Serial.println(" Zone-03 "); 
			HVAC_set = SmartZone_Mode_Update(myData.Climate_Mode, Zone3_Cool, Zone3_Heat, Zone3_Relay);		
		} // myData.NSPanel_ID 03 //

		if (  myData.NSPanel_ID == 0x3034 ){ 
			Serial.println(" Zone-04 "); 
			HVAC_set = SmartZone_Mode_Update(myData.Climate_Mode, Zone4_Cool, Zone4_Heat, Zone4_Relay);
		} // myData.NSPanel_ID 04 //
		
		if (  myData.NSPanel_ID == 0x3035 ){ 
			Serial.println(" Zone-05 "); 
			HVAC_set = SmartZone_Mode_Update(myData.Climate_Mode, Zone5_Cool, Zone5_Heat, Zone5_Relay);
		} // myData.NSPanel_ID 05 //

		if (  myData.NSPanel_ID == 0x3036 ){ 
			Serial.println(" Zone-06 "); 
			HVAC_set = SmartZone_Mode_Update(myData.Climate_Mode, Zone6_Cool, Zone6_Heat, Zone6_Relay);
		} // myData.NSPanel_ID 06 //

		if (  myData.NSPanel_ID == 0x3037 ){ 
			Serial.println(" Zone-07 "); 
			HVAC_set = SmartZone_Mode_Update(myData.Climate_Mode, Zone7_Cool, Zone7_Heat, Zone7_Relay);
		} // myData.NSPanel_ID 07 //

		if (  myData.NSPanel_ID == 0x3038 ){ 
			Serial.println(" Zone-08 "); 
			HVAC_set = SmartZone_Mode_Update(myData.Climate_Mode, Zone8_Cool, Zone8_Heat, Zone8_Relay);
		} // myData.NSPanel_ID 08 //

      Serial.print("Climate Mode : ");
	  Serial.println((char)myData.Climate_Mode);
      } // myData.NSPanel_ST //
	  Serial.print("Sign by: ");
    Serial.println(myData.NSPanel_sign);
  //delay(100); 
  ////////////////////////////////////////////////////////////////////////////////////
  WirelessDataReceived = false;
  } // WirelessDataReceived //
 
 

if ( MainZone_set == true){ // Main Zone Climate Raceiver
    uint8_t MainZone_vec = 0;	// OFF = '0'; Fan = 'F'; Cool = 'C'; Heat = 'H';
	Serial.println(" Main Zone "); 
	HVAC_set = false;
    delay(50);

	MainZoneRegister[MainZone_FAN] = digitalRead(In_FAN);		// FAN
	MainZoneRegister[MainZone_Cool1] = digitalRead(In_Cool1);	// Cool-1
	MainZoneRegister[MainZone_Cool2] = digitalRead(In_Cool2);	// Cool-2
	MainZoneRegister[MainZone_Heat] = digitalRead(In_Heat);		// Heat
	MainZoneRegister[MainZone_Dehum] = digitalRead(In_Dehum);	// Dehum
    
	MainZone_vec = MainZoneRegister[MainZone_FAN] | MainZoneRegister[MainZone_Cool1] | MainZoneRegister[MainZone_Cool2] | MainZoneRegister[MainZone_Heat];

	if ( MainZone_vec == true) { // FAN

		if (MainZoneRegister[MainZone_FAN] == true) { // FAN
			MainZone_mode = 'F';
		}   
  		if (MainZoneRegister[MainZone_Cool1] == true) { // Cool-1
			MainZone_mode = 'C';
			HVAC_Cool_Power = 'C';
		} 
		if (MainZoneRegister[MainZone_Cool2] == true) { // Cool-2
			MainZone_mode = 'C';			
			HVAC_Cool_Power = 'K';
		}
		if (MainZoneRegister[MainZone_Heat] == true) { // Heat
			MainZone_mode = 'H';	
		} 
	} else {
		MainZone_mode = '0';	
	}
	if (MainZoneRegister[MainZone_Dehum] == true) { // Dehum 
	//MainZoneClimate_Mode = 'C'; // Optional //
	HVAC_Cool_Power = 'D';   
	//ic1_pcf8575.digitalWrite(Relay_Comp, HIGH);	//Relay = 1	  
	Serial.println("MainZone Dehum ON");
	} // Dehum
	//HVAC_set = true;

	HVAC_set = SmartZone_Mode_Update(MainZone_mode, Zone1_Cool, Zone1_Heat, Zone1_Relay);
	
	MainZone_set = false;
} 
  
/////////////////////////////////////////////////////////////////////   
  HVAC_Core_Update();  
///////////////////////////////////////////////////////////////////// 
  // AUX
  if (Triggered_AUX) {
    Triggered_AUX = false;
    Serial.println("AUX triggered");
  }

  // 24VAC
  if (Triggered_24VAC) {
    Triggered_24VAC = false;
    Serial.println("24VAC triggered");
  }

/////////////////////////////////////////////////////////////// 

}

String getInterfaceMacAddress(esp_mac_type_t interface) {
  String mac = "";
  unsigned char mac_base[6] = {0};
  if (esp_read_mac(mac_base, interface) == ESP_OK) {
    char buffer[18];  // 6*2 characters for hex + 5 characters for colons + 1 character for null terminator
    sprintf(buffer, "%02X:%02X:%02X:%02X:%02X:%02X", mac_base[0], mac_base[1], mac_base[2], mac_base[3], mac_base[4], mac_base[5]);
    mac = buffer;
  }
  return mac;
}


/*//
void BuckUP_WirelessDataReceived(){
  if ( WirelessDataReceived == true ){ 
  {
	HVAC_set = false;
    Serial.print("String: ");
    Serial.print(myData.NSPanel_Name);
    //Serial.println();
    if ( myData.NSPanel_ST == 0x5A52 | myData.NSPanel_ST == 0x7A72 ){ //Serial.println("Sensor type : Zone Relay");
	
	  if (  myData.NSPanel_ID == 0x3031 ){ 
	    Serial.println(" Zone-01 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone1_Relay] = 0;
            Zone_Heat_Register[Zone1_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone1_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone1_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone1_Relay, LOW); 
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone1_Relay] = 1;
            Zone_Heat_Register[Zone1_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone1_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone1_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone1_Relay] = 0;
            Zone_Heat_Register[Zone1_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone1_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone1_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 01 //

	  if (  myData.NSPanel_ID == 0x3032 ){ 
	    Serial.println(" Zone-02 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone2_Relay] = 0;
            Zone_Heat_Register[Zone2_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone2_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone2_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone2_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone2_Relay] = 1;
            Zone_Heat_Register[Zone2_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone2_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone2_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone2_Relay] = 0;
            Zone_Heat_Register[Zone2_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone2_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone2_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 02 //

	  if (  myData.NSPanel_ID == 0x3033 ){ 
	    Serial.println(" Zone-03 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone3_Relay] = 0;
            Zone_Heat_Register[Zone3_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone3_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone3_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone3_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone3_Relay] = 1;
            Zone_Heat_Register[Zone3_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone3_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone3_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone3_Relay] = 0;
            Zone_Heat_Register[Zone3_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone3_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone3_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 03 //

	  if (  myData.NSPanel_ID == 0x3034 ){ 
	    Serial.println(" Zone-04 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone4_Relay] = 0;
            Zone_Heat_Register[Zone4_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone4_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone4_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone4_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone4_Relay] = 1;
            Zone_Heat_Register[Zone4_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone4_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone4_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone4_Relay] = 0;
            Zone_Heat_Register[Zone4_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone4_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone4_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 04 //

	  if (  myData.NSPanel_ID == 0x3035 ){ 
	    Serial.println(" Zone-05 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone5_Relay] = 0;
            Zone_Heat_Register[Zone5_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone5_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone5_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone5_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone5_Relay] = 1;
            Zone_Heat_Register[Zone5_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone5_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone5_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone5_Relay] = 0;
            Zone_Heat_Register[Zone5_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone5_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone5_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 05 //

	  if (  myData.NSPanel_ID == 0x3036 ){ 
	    Serial.println(" Zone-06 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone6_Relay] = 0;
            Zone_Heat_Register[Zone6_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone6_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone6_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone6_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone6_Relay] = 1;
            Zone_Heat_Register[Zone6_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone6_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone6_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone6_Relay] = 0;
            Zone_Heat_Register[Zone6_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone6_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone6_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 06 //

	  if (  myData.NSPanel_ID == 0x3037 ){ 
	    Serial.println(" Zone-07 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone7_Relay] = 0;
            Zone_Heat_Register[Zone7_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone7_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone7_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone7_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone7_Relay] = 1;
            Zone_Heat_Register[Zone7_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone7_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone7_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone7_Relay] = 0;
            Zone_Heat_Register[Zone7_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone7_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone7_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 07 //

	  if (  myData.NSPanel_ID == 0x3038 ){ 
	    Serial.println(" Zone-08 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone8_Relay] = 0;
            Zone_Heat_Register[Zone8_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone8_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone8_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone8_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone8_Relay] = 1;
            Zone_Heat_Register[Zone8_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone8_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone8_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone8_Relay] = 0;
            Zone_Heat_Register[Zone8_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone8_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone8_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 08 //

      Serial.print("Climate Mode : ");
	  Serial.println((char)myData.Climate_Mode);
      } // myData.NSPanel_ST //
	  Serial.print("Sign by: ");
    Serial.println(myData.NSPanel_sign);
  } //delay(100); 
  ////////////////////////////////////////////////////////////////////////////////////
  WirelessDataReceived = false;
  } // WirelessDataReceived //
 	
}

void BuckUP_loop() {
/***  Relays PinOut Debuging *************************************************** /		
  uint8_t serial_EOL = false;
  uint32_t number = 0;
  // Wait for Enter
  // Read all available bytes
  /** /
  

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
    //sendData();
    //msg_cnt++;
    //lastTime = millis();
    //delay(5000);
      digitalWrite(SPI_LED, LOW);
      Serial.println();
      Serial.print("Echo: "); 
      buf_len = idx;
      for ( idx = 0; idx<=buf_len; idx++ ) { Serial.print( lineBuf[idx] ); }
      Serial.println();

      // Reset buffer for next line
      serial_EOL == true;
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
    } /** /
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
  //yield();  /** /
  if (serial_EOL == true){ 
  Serial.print("Converting DATA ... ");
    idx = 0;
 //   while (lineBuf[idx] != '#'){   //////// ic2_pcf8575.digitalWrite(Zone1_Cool, HIGH); ic1_pcf8575.digitalWrite(Zone1_Relay, LOW); 			// Relay and LED is OFF
     if ( ((lineBuf[idx] == 'Z') | (lineBuf[idx] == 'z')) ){ 
	    if (lineBuf[idx+1] == '1'){
		    if ( ((lineBuf[idx+2] == 'C') | (lineBuf[idx] == 'c')) ){
			   ic2_pcf8575.digitalWrite(Zone1_Cool, ( lineBuf[idx+3] == '0' ? HIGH : LOW ) );
			}
		    if ( ((lineBuf[idx+2] == 'H') | (lineBuf[idx] == 'h')) ){
			   ic2_pcf8575.digitalWrite(Zone1_Heat,  ( lineBuf[idx+3] == '0' ? HIGH : LOW ) );
			}		
		    if ( ((lineBuf[idx+2] == 'R') | (lineBuf[idx] == 'r')) ){
			   ic1_pcf8575.digitalWrite(Zone1_Relay,  ( lineBuf[idx+3] == '0' ? LOW : HIGH ) );
			}		
		}
		if (lineBuf[idx+1] == '2') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone2_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone2_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone2_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
		if (lineBuf[idx+1] == '3') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone3_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone3_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone3_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
		if (lineBuf[idx+1] == '4') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone4_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone4_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone4_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
		if (lineBuf[idx+1] == '5') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone5_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone5_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone5_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
		if (lineBuf[idx+1] == '6') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone6_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone6_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone6_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
		if (lineBuf[idx+1] == '7') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone7_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone7_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone7_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
		if (lineBuf[idx+1] == '8') {
			if (lineBuf[idx+2] == 'C' || lineBuf[idx] == 'c') {
				ic2_pcf8575.digitalWrite(Zone8_Cool, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'H' || lineBuf[idx] == 'h') {
				ic2_pcf8575.digitalWrite(Zone8_Heat, (lineBuf[idx+3] == '0' ? HIGH : LOW));
			}
			if (lineBuf[idx+2] == 'R' || lineBuf[idx] == 'r') {
				ic1_pcf8575.digitalWrite(Zone8_Relay, (lineBuf[idx+3] == '0' ? LOW : HIGH));
			}
		}
	 }
	 
     if ( ((lineBuf[idx] == 'C') | (lineBuf[idx] == 'c')) ){ 
		    if ( lineBuf[idx+1] == '1'){
			   ic1_pcf8575.digitalWrite(Relay_Cool1, ( lineBuf[idx+2] == '0' ? LOW : HIGH) );
			}
		    if ( lineBuf[idx+1] == '2'){
			   ic1_pcf8575.digitalWrite(Relay_Cool2,  ( lineBuf[idx+2] == '0' ? LOW : HIGH) );
			}	
		    if ( lineBuf[idx+1] == 'P'){
			   ic1_pcf8575.digitalWrite(Relay_Comp,  ( lineBuf[idx+2] == '0' ? LOW : HIGH) );
			}				
		}
		
     if ( ((lineBuf[idx] == 'H') | (lineBuf[idx] == 'h')) ){ 
		ic1_pcf8575.digitalWrite(Relay_Heat1, ( lineBuf[idx+1] == '0' ? LOW : HIGH ) );
		}
		
     if ( ((lineBuf[idx] == 'F') | (lineBuf[idx] == 'f')) ){ 
		ic1_pcf8575.digitalWrite(Relay_FAN, ( lineBuf[idx+1] == '0' ? LOW : HIGH ) );
		}
		
     if ( ((lineBuf[idx] == 'A') | (lineBuf[idx] == 'a')) ){ 
		    if ( lineBuf[idx+1] == '1'){
			   ic1_pcf8575.digitalWrite(Relay_Aux1, ( lineBuf[idx+2] == '0' ? LOW : HIGH) );
			}
		    if ( lineBuf[idx+1] == '2'){
			   ic1_pcf8575.digitalWrite(Relay_Aux2,  ( lineBuf[idx+2] == '0' ? LOW : HIGH) );
			}	
		    if ( lineBuf[idx+1] == '3'){
			   ic1_pcf8575.digitalWrite(Relay_Aux3,  ( lineBuf[idx+2] == '0' ? LOW : HIGH) );
			}
			if ( ((lineBuf[idx+1] == 'L') | (lineBuf[idx+1] == 'l')) & ((lineBuf[idx+2] == 'L') | (lineBuf[idx+2] == 'l')) ){
				if ( lineBuf[idx+3] == '0'){
					for(int i=0;i<16;i++) {
						ic1_pcf8575.digitalWrite(i, LOW);
						ic2_pcf8575.digitalWrite(i, HIGH);
					}  
				} else {            
					for(int i=0;i<16;i++) {
						ic1_pcf8575.digitalWrite(i, HIGH);
						ic2_pcf8575.digitalWrite(i, LOW);
					}
				}
			} 
	 }

    if ( lineBuf[idx] == '?'){ 
	   Serial.println("Imput DATA , format type Z1C*,F*,C1*,C2*,H*,CP* ..."); lineBuf[idx]= 0; 
	}
	
  serial_EOL = false;
  for ( idx = 0; idx<8; idx++ ) { lineBuf[idx] == '*'; }
  Serial.println("Done"); digitalWrite(SPI_LED, HIGH);
  }
//delay(100);
/************************************************************************ /	




  if ( WirelessDataReceived == true ){ 
  {
	HVAC_set = false;
    Serial.print("String: ");
    Serial.print(myData.NSPanel_Name);
    //Serial.println();
    if ( myData.NSPanel_ST == 0x5A52 | myData.NSPanel_ST == 0x7A72 ){ //Serial.println("Sensor type : Zone Relay");
	
	  if (  myData.NSPanel_ID == 0x3031 ){ 
	    Serial.println(" Zone-01 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone1_Relay] = 0;
            Zone_Heat_Register[Zone1_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone1_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone1_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone1_Relay, LOW); 
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone1_Relay] = 1;
            Zone_Heat_Register[Zone1_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone1_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone1_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone1_Relay] = 0;
            Zone_Heat_Register[Zone1_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone1_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone1_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 01 //

	  if (  myData.NSPanel_ID == 0x3032 ){ 
	    Serial.println(" Zone-02 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone2_Relay] = 0;
            Zone_Heat_Register[Zone2_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone2_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone2_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone2_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone2_Relay] = 1;
            Zone_Heat_Register[Zone2_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone2_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone2_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone2_Relay] = 0;
            Zone_Heat_Register[Zone2_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone2_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone2_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 02 //

	  if (  myData.NSPanel_ID == 0x3033 ){ 
	    Serial.println(" Zone-03 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone3_Relay] = 0;
            Zone_Heat_Register[Zone3_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone3_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone3_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone3_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone3_Relay] = 1;
            Zone_Heat_Register[Zone3_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone3_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone3_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone3_Relay] = 0;
            Zone_Heat_Register[Zone3_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone3_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone3_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 03 //

	  if (  myData.NSPanel_ID == 0x3034 ){ 
	    Serial.println(" Zone-04 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone4_Relay] = 0;
            Zone_Heat_Register[Zone4_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone4_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone4_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone4_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone4_Relay] = 1;
            Zone_Heat_Register[Zone4_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone4_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone4_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone4_Relay] = 0;
            Zone_Heat_Register[Zone4_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone4_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone4_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 04 //

	  if (  myData.NSPanel_ID == 0x3035 ){ 
	    Serial.println(" Zone-05 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone5_Relay] = 0;
            Zone_Heat_Register[Zone5_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone5_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone5_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone5_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone5_Relay] = 1;
            Zone_Heat_Register[Zone5_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone5_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone5_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone5_Relay] = 0;
            Zone_Heat_Register[Zone5_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone5_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone5_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 05 //

	  if (  myData.NSPanel_ID == 0x3036 ){ 
	    Serial.println(" Zone-06 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone6_Relay] = 0;
            Zone_Heat_Register[Zone6_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone6_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone6_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone6_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone6_Relay] = 1;
            Zone_Heat_Register[Zone6_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone6_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone6_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone6_Relay] = 0;
            Zone_Heat_Register[Zone6_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone6_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone6_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 06 //

	  if (  myData.NSPanel_ID == 0x3037 ){ 
	    Serial.println(" Zone-07 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone7_Relay] = 0;
            Zone_Heat_Register[Zone7_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone7_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone7_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone7_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone7_Relay] = 1;
            Zone_Heat_Register[Zone7_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone7_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone7_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone7_Relay] = 0;
            Zone_Heat_Register[Zone7_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone7_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone7_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 07 //

	  if (  myData.NSPanel_ID == 0x3038 ){ 
	    Serial.println(" Zone-08 "); 
	    if ( myData.Climate_Mode == '0' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
            Zone_Cool_Register[Zone8_Relay] = 0;
            Zone_Heat_Register[Zone8_Relay] = 0;			
			
			ic2_pcf8575.digitalWrite(Zone8_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone8_Heat, HIGH);
			ic1_pcf8575.digitalWrite(Zone8_Relay, LOW);
			HVAC_set = true;		
		}  // Relay and LED is OFF
		
	    if ( myData.Climate_Mode == 'C' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone8_Relay] = 1;
            Zone_Heat_Register[Zone8_Relay] = 0;			

			ic2_pcf8575.digitalWrite(Zone8_Cool, LOW); 
			ic2_pcf8575.digitalWrite(Zone8_Heat, HIGH);
			
			HVAC_set = true;
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'C'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'H' ) { 
				HVAC_nextmode = 'C'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}

		}  // Relay and LED is ON
		
	    if ( myData.Climate_Mode == 'H' ){ Serial.print(" *Climate Mode: ");    Serial.println((char)myData.Climate_Mode);
			Zone_Cool_Register[Zone8_Relay] = 0;
            Zone_Heat_Register[Zone8_Relay] = 1;			

			ic2_pcf8575.digitalWrite(Zone8_Cool, HIGH); 
			ic2_pcf8575.digitalWrite(Zone8_Heat, LOW); 
			
			HVAC_set = true;			
			
			if ( HVAC_mode == '0' ){ 
				HVAC_mode = 'H'; 
				Serial.print(" *HVAC_mode: ");    
				Serial.println((char)HVAC_mode);
			}			
			if ( HVAC_mode == 'C' ) { 
				HVAC_nextmode = 'H'; 
				Serial.print("*HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
			}		
			
		}  // Relay and LED is ON		
	   } // myData.NSPanel_ID 08 //

      Serial.print("Climate Mode : ");
	  Serial.println((char)myData.Climate_Mode);
      } // myData.NSPanel_ST //
	  Serial.print("Sign by: ");
    Serial.println(myData.NSPanel_sign);
  } //delay(100); 
  ////////////////////////////////////////////////////////////////////////////////////
  WirelessDataReceived = false;
  } // WirelessDataReceived //
 
 

if ( MainZone_set == True){
	Serial.println(" Main Zone "); 
	if (flag_FAN) {
		flag_FAN = false;
		Serial.println("Interrupt: FAN triggered");
	}
	if (flag_Cool1) {
		flag_Cool1 = false;
		Serial.println("Interrupt: Cool1 triggered");
	}
	if (flag_Cool2) {
		flag_Cool2 = false;
		Serial.println("Interrupt: Cool2 triggered");
	}
	if (flag_Heat) {
		flag_Heat = false;
		Serial.println("Interrupt: Heat triggered");
	}
	MainZone_set = false;
  } 
 
 
 
 
  if ( HVAC_set == true ){ Serial.print("HVAC_SET: ");    Serial.println(HVAC_set);
		ix = 0;
		Cool_Relay_cnt = 0;
		Heat_Relay_cnt = 0;
		HVAC_prevmode = 0;
		HVAC_Cool_vect = 0;
		HVAC_Heat_vect = 0;
		
		Serial.print("HVAC_Register:");
        for(idx=0;idx<8;idx++) { 
			HVAC_Cool_vect|=Zone_Cool_Register[idx]; Serial.print(" C");    Serial.print(Zone_Cool_Register[idx]);
        } Serial.println();	
		Serial.print("HVAC_Register:");
        for(idx=0;idx<8;idx++) { 
			HVAC_Heat_vect|=Zone_Heat_Register[idx]; Serial.print(" H");    Serial.print(Zone_Heat_Register[idx]); 
        } Serial.println();	
		
		Serial.print("#HVAC_Cool_vect: ");    Serial.println(HVAC_Cool_vect);
		Serial.print("#HVAC_Heat_vect: ");    Serial.println(HVAC_Heat_vect);
		
//	if ( HVAC_Cool_vect == 1 ) { 
//		HVAC_mode = 'C'; HVAC_ON = true;		Serial.print("#HVAC_mode: ");    Serial.println((char)HVAC_mode);
//	}	
//	if ( HVAC_Heat_vect == 1 ) { 
//		HVAC_mode = 'H'; HVAC_ON = true;		Serial.print("#HVAC_mode: ");    Serial.println((char)HVAC_mode);
//	}	
	if (( HVAC_Heat_vect == 0 )&( HVAC_nextmode == 'C' )) { 
		HVAC_mode = 'C'; 
		HVAC_nextmode = '0'; 
		HVAC_ON = true;		
		Serial.print("@HVAC_mode: "); Serial.println((char)HVAC_mode);
		Serial.print("@HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
	}	
	if (( HVAC_Cool_vect == 0 )&( HVAC_nextmode == 'H' ))  { 
		HVAC_mode = 'H';
		HVAC_nextmode = '0'; 
		HVAC_ON = true;		
		Serial.print("@HVAC_mode: "); Serial.println((char)HVAC_mode);
		Serial.print("@HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
	}	
	if (( HVAC_Cool_vect == 0 )&( HVAC_Heat_vect == 0 )) { 
		HVAC_mode = '0';
		HVAC_nextmode = '0'; 		
		HVAC_ON = false;		
		Serial.print("!HVAC_mode: "); Serial.println((char)HVAC_mode);
		Serial.print("!HVAC_nextmode: "); Serial.println((char)HVAC_nextmode);
	}	
	
	if ( HVAC_mode == '0' ){ Serial.print("$HVAC_mode: ");    Serial.println((char)HVAC_mode);
		ic1_pcf8575.digitalWrite(Relay_FAN, LOW);	//Relay = 0
		ic1_pcf8575.digitalWrite(Relay_Cool1, LOW);	//Relay = 0
		ic1_pcf8575.digitalWrite(Relay_Cool2, LOW);	//Relay = 0
		ic1_pcf8575.digitalWrite(Relay_Heat1, LOW);	//Relay = 0		
		ic1_pcf8575.digitalWrite(Relay_Comp, LOW);	//Relay = 0	
	}
    if ( HVAC_mode == 'C' ){ Serial.print("$HVAC_mode: ");    Serial.println((char)HVAC_mode);
		ix = 0;
		Cool_Relay_cnt = 0;
		ic1_pcf8575.digitalWrite(Relay_FAN, HIGH);	//Relay = 1
		ic1_pcf8575.digitalWrite(Relay_Cool1, HIGH);	//Relay = 1
		ic1_pcf8575.digitalWrite(Relay_Cool2, LOW);	//Relay = 0
		ic1_pcf8575.digitalWrite(Relay_Heat1, LOW);	//Relay = 0		
		ic1_pcf8575.digitalWrite(Relay_Comp, HIGH);	//Relay = 1	
			
        for(idx=0;idx<8;idx++) {
			Serial.print("ZC ");    Serial.print(Zone_Cool_Register[idx]); Serial.print(" ix: "); Serial.println(ix);
			if ( Zone_Cool_Register[idx] == 1 ){
				ic1_pcf8575.digitalWrite(idx, HIGH);	//Relay//1			
				Cool_Relay_cnt++;
			} // else {
//				ic1_pcf8575.digitalWrite(idx, LOW);	//Relay//1	
//			}
		ix +=2; 
        } Serial.println();
		
		if ( HVAC_ON == false ){ 
			HVAC_ON = true; 
			}		
		
	}
	
    if ( HVAC_mode == 'H' ){ Serial.print("$HVAC_mode: ");    Serial.println((char)HVAC_mode);
	    ix = 1;
		Heat_Relay_cnt = 0;
		ic1_pcf8575.digitalWrite(Relay_FAN, HIGH);	//Relay = 1
		ic1_pcf8575.digitalWrite(Relay_Cool1, LOW);	//Relay = 0
		ic1_pcf8575.digitalWrite(Relay_Cool2, LOW);	//Relay = 0
		ic1_pcf8575.digitalWrite(Relay_Heat1, HIGH);	//Relay = 1		
		ic1_pcf8575.digitalWrite(Relay_Comp, LOW);	//Relay = 0	
        for(idx=0;idx<8;idx++) {
			Serial.print("ZH ");    Serial.print(Zone_Heat_Register[idx]); Serial.print(" ix: "); Serial.println(ix);
			if ( Zone_Heat_Register[idx] == 1 ){
				ic1_pcf8575.digitalWrite(idx, HIGH);	//Relay = 1			
				Heat_Relay_cnt++;
			} // else {
//				ic1_pcf8575.digitalWrite(idx, LOW);	//Relay//1	
//			}
		ix +=2; 
        } Serial.println();
		if ( HVAC_ON == false ){ 
			HVAC_ON = true; 
			}		
		
	}  
  
  HVAC_set = false; 
  }	  // HVAC_set //  
///////////////////////////////////////////////////////////////////// 
   if (flag_Dehum) {
		flag_Dehum = false;
		Serial.println("Interrupt: Dehum triggered");
	}
  if (flag_AUX) {
		flag_AUX = false;
		Serial.println("Interrupt: AUX triggered");
	}
  if (flag_24VAC) {
    flag_24VAC = false;
    Serial.println("Interrupt: 24VAC detected");
  }
/////////////////////////////////////////////////////////////// 

}

/**/