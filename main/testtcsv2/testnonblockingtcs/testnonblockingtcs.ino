#include <Arduino.h>
#include <Wire.h>
#include "TCS34725AutoGain.h"

TCS34725 tcs1, tcs2;

#define TCAADR 0x70
#define ONBOARDLEDPIN 25

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
    // pinMode(ONBOARDLEDPIN, OUTPUT);
}

void loop(void)
{  
    // digitalWrite(ONBOARDLEDPIN, HIGH);
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


/*
White and white
First TCS selected: 
Color Temp : 5629.63
Lux        : 1298.90
R          : 12.47
G          : 18.67
B          : 16.28
C          : 535
Second TCS selected: 
Color Temp : 6355.55
Lux        : 906.03
R          : 9.64
G          : 20.93
B          : 18.68
C          : 367


Black and black
First TCS selected: 
Color Temp : 6420.20
Lux        : 226.02
R          : 8.82
G          : 19.04
B          : 17.67
C          : 96
Second TCS selected: 
Color Temp : 7206.26
Lux        : 195.82
R          : 6.80
G          : 21.29
B          : 19.56
C          : 81

Black and white (2 then 4, ie right then left)
First TCS selected: 
Color Temp : 6360.57
Lux        : 206.75
R          : 9.16
G          : 19.33
B          : 17.81
C          : 87
Second TCS selected: 
Color Temp : 6317.06
Lux        : 910.45
R          : 9.77
G          : 21.21
B          : 18.57
C          : 365

White and black
First TCS selected: 
Color Temp : 5658.20
Lux        : 1220.21
R          : 12.32
G          : 18.64
B          : 16.36
C          : 504
Second TCS selected: 
Color Temp : 7406.79
Lux        : 201.34
R          : 6.39
G          : 21.74
B          : 20.03
C          : 83



*/