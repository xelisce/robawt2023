/**
 * TCA9548 I2CScanner.ino -- I2C bus scanner for Arduino
 *
 * Based on https://playground.arduino.cc/Main/I2cScanner/
 *
 */

#include "Wire.h"

#define TCAADDR 0x70

#define WIRE Wire

void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  WIRE.beginTransmission(TCAADDR);
  WIRE.write(1 << i);
  WIRE.endTransmission();  
}


// standard Arduino setup()
void setup()
{
    while (!Serial);
    delay(1000);

    WIRE.setSDA(6);
    WIRE.setSCL(7);
    WIRE.begin();
    
    Serial.begin(9600);
    Serial.println("\nTCAScanner ready!");
    
    for (uint8_t t=0; t<8; t++) {
      tcaselect(t);
      Serial.print("TCA Port #"); Serial.println(t);

      for (uint8_t addr = 0; addr<=127; addr++) {
        if (addr == TCAADDR) continue;

        WIRE.beginTransmission(addr);
        if (!WIRE.endTransmission()) {
          Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
        }
      }
    }
    Serial.println("\ndone");
}

void loop() 
{
}
