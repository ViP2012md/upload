// ESP8266 Arduino code: read a line until Enter (CR/LF), store in buffer,
// then send it back over Serial.

#include <Arduino.h>

#define LINE_BUF_SIZE 128

char lineBuf[LINE_BUF_SIZE];
size_t idx = 0;

void setup() {
  Serial.begin(115200);
  // No need for while(!Serial) on ESP8266
  delay(50); 
  Serial.println();
  Serial.println("Type text and press Enter. I will echo it back.");
}

void loop() {
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
}