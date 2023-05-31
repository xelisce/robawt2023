#include <Arduino.h>
#include <Wire.h>
#include "Vroom.h"
#include <Servo.h>
#include "VL53L1X.h"

#define SDAPIN 4
#define SCLPIN 5
#define TCAADDR 0x70

VL53L1X sensor;

void setup(){
    Serial.begin(9600);

    Wire.setSDA(SDAPIN);
    Wire.setSCL(SCLPIN);
    Wire.begin();
    Wire.setClock(400000);

    // tcaselect(5);
    // while(!sensor.init()) {
    //     Serial.println("L");
    // }
    // sensor.startContinuous();

    for (int i = 0; i<8; i++) 
    {
        tcaselect(i);
        sensor.setTimeout(500);
        if (sensor.init()) {
            sensor.startContinuous();
            a = i;
            // Serial.print("Sensor at pin ");
            // Serial.print(i);
            // Serial.println("L0X failed to initialise");
        }
        // sensor.startContinuous();
    }
}
int reading;

void loop(){
    
    // for (int i = 0; i<8; i++) 
    // {
    //     tcaselect(i);
    //     sensor.setTimeout(500);
    //     if (sensor.init()) {
    //         sensor.startContinuous();
    //         a = i;
    //         // Serial.print("Sensor at pin ");
    //         // Serial.print(i);
    //         // Serial.println("L0X failed to initialise");
    //     }
    //     // sensor.startContinuous();
    // }
    tcaselect(a);
    // for (int i = 0; i < 8; i++) 
    // {
    //     tcaselect(i);
    //     if(sensor.available()) {
    //         reading = sensor.readRangeMillimeters();
    //         Serial.print("Sensor ");
    //         Serial.print(i);
    //         Serial.print("----");
    //         Serial.println(reading);
    //     }
    // }
    Serial.println(sensor.readRangeMillimeters());
}

void tcaselect(uint8_t i)  //I2C Multiplexer: TCA9548A
{
    if (i >= 0 && i <= 7) {
        Wire.beginTransmission(TCAADDR);
        Wire.write(1 << i);
        Wire.endTransmission();
    }
}