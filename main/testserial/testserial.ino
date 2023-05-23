#include <Arduino.h>

#define default_serial 1
#define test_pi 1
#define test_teensy 0
#define print 0

const int TX1PIN = 15,
  RX1PIN = 14,
  TX0PIN = 16,
  RX0PIN = 17;
  
void setup() 
{
  #if default_serial
  Serial.begin(9600);
  while(!Serial) delay(10);
  Serial.println("USB Serial initialised!");
  #endif

  #if test_pi
  Serial1.setRX(RX0PIN);
  Serial1.setTX(TX0PIN);
  Serial1.begin(9600);
  // while (!Serial1) delay(10);
  Serial.println("Pi serial initialised");
  #endif

  #if test_teensy
  Serial2.setRX(RX1PIN);
  Serial2.setRX(TX1PIN);
  Serial2.begin(9600);
  // while (!Serial2) delay(10); 
  Serial.println("Teensy serial initialised");
  #endif
}

void loop() 
{
  #if default_serial && print
  Serial.println(1);
  #endif

  #if test_pi && print
  Serial1.println(1);
  #endif

  #if test_teensy && print
  Serial2.println(1);
  #endif

  delay(1000);
}
