#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

String readLineWithTimeout(unsigned long timeout_ms=500) {
  String s = "";
  unsigned long start = millis();
  while (true) {
    if (Serial.available()) {
      char c = (char)Serial.read();
      if (c == '\n') break;
      s += c;
    } else {
      if (millis() - start > timeout_ms) break;
      delay(1);
    }
  }
  return s;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  lcd.begin(16, 2);
  lcd.setRGB(0, 128, 255);
  lcd.print("Waiting for");
  lcd.setCursor(0, 1);
  lcd.print("joint_states...");
}

void loop() {
  String l1 = readLineWithTimeout();
  String l2 = readLineWithTimeout();
  if (l1.length() == 0 && l2.length() == 0) {
    delay(100);
    return;
  }
  lcd.clear();
  lcd.setCursor(0, 0);
  if (l1.length() > 16) l1 = l1.substring(0, 16);
  lcd.print(l1);
  lcd.setCursor(0, 1);
  if (l2.length() > 16) l2 = l2.substring(0, 16);
  lcd.print(l2);
  delay(100);
}

