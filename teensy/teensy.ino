#include <Arduino.h>

void setup() {
  // Serial.begin(9600);
  // Serial.println("USB serial initialised.");

  Serial1.begin(9600);
  Serial.println("Teensy-Pico serial initialised.");
}

void loop() {
  Serial1.println(1);
  // Serial.println(1);
  delay(1000);
}
