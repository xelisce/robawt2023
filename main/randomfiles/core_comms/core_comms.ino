#include <Arduino.h>
#include "VL53L0X.h"

VL53L0X sensor;

#define TCAADDR 0x70
const int TNYPIN1 = 9,
    TNYPIN2 = 8,
    TX0PIN = 16,
    RX0PIN = 17,
    SDAPIN = 4,
    SCLPIN = 5,
    SWTPIN = 28,
    LEDPIN = 3,
    ONBOARDLEDPIN = 25;

uint32_t current = 0;
uint32_t front_dist = 0;
long startTimer;

void setup() {
    Serial.begin(9600);
}

void loop() {
    startTimer = micros();
    multicoreReceive();
    Serial.print("Time taken: ");
    Serial.println(micros()-startTimer);
    Serial.print("Received: ");
    Serial.println(front_dist);
    
}

void setup1() {
    Serial.begin(9600);
    Serial.println("USB serial initialised");

    Wire.setSDA(SDAPIN);
    Wire.setSCL(SCLPIN);
    Wire.begin();
    Wire.setClock(400000); 

    tcaselect(4);
    sensor.setTimeout(500);
    while (!sensor.init()) {
        Serial.println("L0X failed to initialise");
    }
    sensor.startContinuous();
}

void loop1() 
{
    tcaselect(4);
    if(sensor.available()) {
        rp2040.fifo.push(sensor.readRangeMillimeters());
        // if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    }

}

void multicoreReceive()
{
    while (rp2040.fifo.available() > 0) {
        front_dist = rp2040.fifo.pop();
    }
}


void tcaselect(uint8_t i)  //Multiplexer: TCA9548A
{
  if (i > 0 && i < 7) {
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
  }
}
