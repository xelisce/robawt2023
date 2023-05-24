#include <Arduino.h>
#include <Wire.h>
#include "TCS34725AutoGain.h"

TCS34725 tcs;

#define TCAADR 0x70
#define L0XADR 0x29
#define L1XADR 0x29
#define TCSADR 0x29

#define TNYPIN1 9
#define TNYPIN2 8
#define TX0PIN 16
#define RX0PIN 17
#define SDAPIN 4
#define SCLPIN 5
#define SDA1PIN 6
#define SCL1PIN 7
#define SWTPIN 28
#define LEDPIN 3
#define ONBOARDLEDPIN 25

void setup() 
{
    Serial.begin(9600);
    Serial.println("Serial intialised");

    Wire1.setSDA(SDA1PIN);
    Wire1.setSCL(SCL1PIN);
    Wire1.begin();
    Wire1.setClock(400000);
    // while (!Wire1.init()) delay(10); 
    Serial.println("TCA2 initialised");

    for (int i=0; i <= 7; i++) {
        tcaselect2(i);
        if (!tcs.attach(Wire1))
            Serial.println("ERROR: TCS34725 NOT FOUND !!!");
        tcs.integrationTime(33); // ms
        tcs.gain(TCS34725::Gain::X01);
        Serial.println("Success!");
    }
    Serial.println("HAHA");
}

void loop() 
{
    // tcaselect2(1);
    // TCS34725::Color color = tcs.color();
    // TCS34725::RawData rawdata = tcs.raw();
    // Serial.print("Color Temp : "); Serial.println(tcs.colorTemperature());
    // Serial.print("Lux        : "); Serial.println(tcs.lux());
    // Serial.print("R          : "); Serial.println(color.r);
    // Serial.print("G          : "); Serial.println(color.g);
    // Serial.print("B          : "); Serial.println(color.b);
    // Serial.print("C          : "); Serial.println(rawdata.c);
    // for (int i=0; i <= 7; i++) {
    //     tcaselect2(i);
    //     TCS34725::Color color = tcs.color();
    //     TCS34725::RawData rawdata = tcs.raw();
    //     Serial.print("Color Temp : "); Serial.println(tcs.colorTemperature());
    //     Serial.print("Lux        : "); Serial.println(tcs.lux());
    //     Serial.print("R          : "); Serial.println(color.r);
    //     Serial.print("G          : "); Serial.println(color.g);
    //     Serial.print("B          : "); Serial.println(color.b);
    //     Serial.print("C          : "); Serial.println(rawdata.c);
    // }
}

void tcaselect(uint8_t i) //I2C Multiplexer: TCA9548A
{
    if (i >= 0 && i <= 7) {
        Wire.beginTransmission(TCAADR);
        Wire.write(1 << i);
        Wire.endTransmission();
    }
}

void tcaselect2(uint8_t i) //I2C Multiplexer: TCA9548A
{
    if (i >= 0 && i <= 7) {
        Wire1.beginTransmission(TCAADR);
        Wire1.write(1 << i);
        Wire1.endTransmission();
    }
}