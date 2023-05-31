#include <Arduino.h>
#include <Wire.h>
#include "Vroom.h"
#include <Servo.h>
#include "VL53L1X.h"
#include "VL53L0X.h"

#define SDAPIN 4
#define SCLPIN 5
#define TCAADDR 0x70

#define l0x 0
#define l1x 1

VL53L0X sensor;
VL53L1X fb_lidar;

int reading;
int fb_reading;

void setup(){
    Serial.begin(9600);

    Wire.setSDA(SDAPIN);
    Wire.setSCL(SCLPIN);
    Wire.begin();
    Wire.setClock(400000);
    
    #if l0x
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
    #endif

    #if l1x
    tcaselect(4);
    fb_lidar.setTimeout(500);
    while (!fb_lidar.init()) Serial.println("FRONT BOTTOM LIDAR FAILED TO INIT");
    fb_lidar.setDistanceMode(VL53L1X::Medium);
    fb_lidar.startContinuous(20);
    #endif

}

void loop(){
    #if l0x
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
    tcaselect(a);
    for (int i = 0; i < 8; i++) 
    {
        tcaselect(i);
        if(sensor.available()) {
            reading = sensor.readRangeMillimeters();
            Serial.print("Sensor ");
            Serial.print(i);
            Serial.print("----");
            Serial.println(reading);
        }
    }
    Serial.println(sensor.readRangeMillimeters());
    #endif

    #if l1x
    tcaselect(4);
    if (fb_lidar.dataReady()){
        fb_reading = fb_lidar.read(false);
    }
    Serial.println(fb_reading);
    #endif
}

void tcaselect(uint8_t i)  //I2C Multiplexer: TCA9548A
{
    if (i >= 0 && i <= 7) {
        Wire.beginTransmission(TCAADDR);
        Wire.write(1 << i);
        Wire.endTransmission();
    }
}