#include <Arduino.h>

void setup() {
    pinMode(15, INPUT);
}

void loop() {
    Serial.println(digitalRead(15));
}