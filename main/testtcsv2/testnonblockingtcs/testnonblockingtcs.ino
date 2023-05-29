#include <Arduino.h>
#include <Wire.h>
#include "TCS34725AutoGain.h"

TCS34725 tcs1, tcs2;

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
    tcaselect2(2);
    if (!tcs1.attach(&Wire1))
        Serial.println("ERROR: TCS34725 NOT FOUND !!!");
    tcs1.integrationTime(33); // ms
    tcs1.gain(TCS34725::Gain::X01);

    tcaselect2(4);
    if (!tcs2.attach(&Wire1)) Serial.println("ERROR: TCS34725 NOT FOUND !!!");
    tcs2.integrationTime(33); // ms
    tcs2.gain(TCS34725::Gain::X01);

}

void loop(void)
{
    
    tcaselect2(2);
    if (tcs1.available()) // if current measurement has done
    {
        Serial.println("First TCS selected: ");
        TCS34725::Color color = tcs1.color();
        TCS34725::RawData rawdata = tcs1.raw();
        Serial.print("Color Temp : "); Serial.println(tcs1.colorTemperature());
        Serial.print("Lux        : "); Serial.println(tcs1.lux());
        Serial.print("R          : "); Serial.println(color.r);
        Serial.print("G          : "); Serial.println(color.g);
        Serial.print("B          : "); Serial.println(color.b);
        Serial.print("C          : "); Serial.println(rawdata.c);
        // Serial.print("Everything : "); Serial.println(rawdata.);
    }

   
    tcaselect2(4);
    if (tcs2.available()) // if current measurement has done
    {
        Serial.println("Second TCS selected: ");
        TCS34725::Color color = tcs2.color();
        TCS34725::RawData rawdata = tcs2.raw();
        Serial.print("Color Temp : "); Serial.println(tcs2.colorTemperature());
        Serial.print("Lux        : "); Serial.println(tcs2.lux());
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