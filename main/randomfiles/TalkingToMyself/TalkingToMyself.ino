// --------------------------------------
// i2c_scanner
//
// Modified from https://playground.arduino.cc/Main/I2cScanner/
// --------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include "VL53L0X.h"

#define SDAPIN 6
#define SCLPIN 7
#define TCAADDR 0x70
#define WIRE Wire1

// VL53L0X lidarsl0x;

void setup() {
  WIRE.setSDA(SDAPIN);
  WIRE.setSCL(SCLPIN);
  WIRE.begin();
  WIRE.setClock(400000); 
  Serial.begin(9600);
  while (!Serial)
     delay(10);
  Serial.println("\nI2C Scanner");

  // tcaselect(3);
  // lidarsl0x.setTimeout(500);
  // while (!lidarsl0x.init()) {
  //     Serial.println("L0X failed to initialise");
  // }
  // lidarsl0x.startContinuous();
}


void loop() {
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    WIRE.beginTransmission(address);
    error = WIRE.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknown error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(2000);           // wait 5 seconds for next scan

  // tcaselect(3);
  // if(lidarsl0x.available()) {
  //     Serial.println(lidarsl0x.readRangeMillimeters());
  // }
}

void tcaselect(uint8_t i)  //I2C Multiplexer: TCA9548A
{
    if (i > 0 && i < 7) {
        Wire.beginTransmission(TCAADDR);
        Wire.write(1 << i);
        Wire.endTransmission();
    }
}