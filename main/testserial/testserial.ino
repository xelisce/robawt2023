#include <Arduino.h>

const int TX0PIN = 16,
  RX0PIN = 17;
  
void setup() {
  Serial.begin(9600);
  while(!Serial) delay(10);
  Serial.println("USB Serial initialised!");
  
  Serial1.setRX(RX0PIN);
  Serial1.setTX(TX0PIN);
  Serial1.begin(9600);
  while (!Serial1) delay(10);
  Serial.println("Pi serial initialised");
}

void loop() {
  if(Serial.available()) {
    Serial2.write(Serial.read());
  }
}
