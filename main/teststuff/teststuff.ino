#include <Arduino.h>
#include <Wire.h>

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10);
}

void loop() {
  Serial.println("Hellos");
}
