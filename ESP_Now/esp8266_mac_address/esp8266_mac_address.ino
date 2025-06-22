/*
  ESP8266 MAC Address printout
  esp8266-mac-address.ino
  Prints MAC Address to Serial Monitor

*/

// Include WiFi Library
#include <espnow.h>
#include <ESP8266WiFi.h>

void setup() {

  pinMode(LED_BUILTIN, OUTPUT);  // Initialize the LED_BUILTIN pin as an output
  // Setup Serial Monitor
  Serial.begin(115200);
  delay(1000);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
    // Print MAC Address to Serial monitor
  Serial.println();
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  delay(2000);  // Wait for 2 seconds
  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED on (Note that LOW is the voltage level
  // but actually the LED is on; this is because
  // it is active low on the ESP-01)
  delay(1000);
}
