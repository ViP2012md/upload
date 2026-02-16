/*
 * IC1 PCF8575 GPIO Port Expand
 * A2A1A0 ----------------- 0 0 1 (0x22)
 * P0     ----------------- Zone-8 Relay Dumper 
 * P1     ----------------- Zone-7 Relay Dumper 
 * P2     ----------------- Zone-6 Relay Dumper 
 * P3     ----------------- Zone-5 Relay Dumper 
 * P4     ----------------- Zone-4 Relay Dumper 
 * P5     ----------------- Zone-3 Relay Dumper 
 * P6     ----------------- Zone-2 Relay Dumper 
 * P7     ----------------- Zone-1 Relay Dumper 
 * P10    ----------------- Relay-5 Compressor
 * P11    ----------------- Relay-4 HEAT
 * P12    ----------------- Relay-3 COOL-2
 * P13    ----------------- Relay-2 COOL-1
 * P14    ----------------- Relay-1 FAN
 * P15    ----------------- NA
 * P16    ----------------- NA
 * P17    ----------------- NA
 *
 * IC2 PCF8575 GPIO Port Expand
 * A2A1A0 ----------------- 0 0 0 (0x20)
 * P0     ----------------- Zone-1 COOL
 * P1     ----------------- Zone-1 HEAT
 * P2     ----------------- Zone-2 COOL
 * P3     ----------------- Zone-2 HEAT
 * P4     ----------------- Zone-3 COOL
 * P5     ----------------- Zone-3 HEAT
 * P6     ----------------- Zone-4 COOL
 * P7     ----------------- Zone-4 HEAT
 * P10    ----------------- Zone-5 COOL
 * P11    ----------------- Zone-5 HEAT
 * P12    ----------------- Zone-6 COOL
 * P13    ----------------- Zone-6 HEAT
 * P14    ----------------- Zone-7 COOL
 * P15    ----------------- Zone-7 HEAT
 * P16    ----------------- Zone-8 COOL
 * P17    ----------------- Zone-8 HEAT
 */ 

#include "Arduino.h"
#include "PCF8575.h"  // https://github.com/xreef/PCF8575_library


// Instantiate Wire for generic use at 400kHz
TwoWire I2Cone = TwoWire(0);
// Set pcf8575 i2c comunication with second Wire using 21 22 as SDA SCL
PCF8575 ic1_pcf8575(&I2Cone,0x20);
PCF8575 ic2_pcf8575(&I2Cone,0x21);

void setup()
{
  Serial.begin(115200);
  TwoWire I2Cone = TwoWire(0);
  // Set pinMode to OUTPUT
  for(int i=0;i<16;i++) {
    ic1_pcf8575.pinMode(i, OUTPUT);
    ic2_pcf8575.pinMode(i, OUTPUT);
  }
  ic1_pcf8575.begin();
  ic2_pcf8575.begin();
}

void loop()
{
  static int pin = 0;
  ic1_pcf8575.digitalWrite(pin, HIGH); ic2_pcf8575.digitalWrite(16-pin, HIGH);
  delay(200);
  ic1_pcf8575.digitalWrite(pin, LOW); ic2_pcf8575.digitalWrite(16-pin, LOW);
  delay(50);
  pin++;
  if (pin > 15) pin = 0;
}
