#include <Arduino.h>
#include <Wire.h>
#include "TCS34725AutoGain.h"

TCS34725 tcs;

#define TCAADR 0x70

void setup(void)
{
    Serial.begin(9600);
    while (!Serial) delay(10);
    // Wire.setSDA(4);
    // Wire.setSCL(5);
    // Wire.begin();
    // Wire.setClock(400000);
    // if (!tcs.attach(Wire))
    //     Serial.println("ERROR: TCS34725 NOT FOUND !!!");
    Wire1.setSDA(6);
    Wire1.setSCL(7);
    Wire1.begin();
    Wire1.setClock(400000);
    tcaselect2(1);
    if (!tcs.attach(Wire1))
        Serial.println("ERROR: TCS34725 NOT FOUND !!!");

    tcs.integrationTime(33); // ms
    tcs.gain(TCS34725::Gain::X01);

}

void loop(void)
{
    tcaselect2(1);
    if (tcs.available()) // if current measurement has done
    {
        TCS34725::Color color = tcs.color();
        TCS34725::RawData rawdata = tcs.raw();
        Serial.print("Color Temp : "); Serial.println(tcs.colorTemperature());
        Serial.print("Lux        : "); Serial.println(tcs.lux());
        Serial.print("R          : "); Serial.println(color.r);
        Serial.print("G          : "); Serial.println(color.g);
        Serial.print("B          : "); Serial.println(color.b);
        Serial.print("C          : "); Serial.println(rawdata.c);
        // Serial.print("Everything : "); Serial.println(rawdata.);

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