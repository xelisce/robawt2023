#include <Arduino.h>
#include <Wire.h>
#include <TCA9548A.h>

TCA9548A mux(6, 7, &Wire1, 400000);

void setup()
{
    mux.begin();
    Serial.begin(9600);
    while (!Serial) delay(10);
    Serial.println("Done");

    if (mux.select(2)) Serial.println("Done2");
}

void loop() {}


// //* LIBRARIES
// #include <Arduino.h>
// #include <Wire.h>
// #include "Adafruit_TCS34725.h"

// //* OBJECT INITIALISATIONS
// Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// //* ADDRESSES
// #define TCAADR 0x70
// #define L0XADR 0x29
// #define L1XADR 0x29
// #define TCSADR 0x29

// //* PINS
// #define LEDPIN 3
// #define SDAPIN 4
// #define SCLPIN 5
// #define SDA1PIN 6
// #define SCL1PIN 7
// #define TX0PIN 16
// #define RX0PIN 17
// #define PICOLEDPIN 25
// #define SWTPIN 28

// void setup() 
// {
//     Serial.begin(9600);
//     while (!Serial) delay(10);
//     Serial.println("Serial intialised");
    
//     Wire1.setSDA(SDA1PIN);
//     Wire1.setSCL(SCL1PIN);
//     Wire1.begin();
//     Wire1.setClock(400000);
//     Serial.println("TCA initialised");

//     tcaselect2(2);
//     if (!tcs.begin(0x29, &Wire1)) {
//         Serial.println("ERROR: TCS34725 NOT FOUND !!!");
//         while (1);
//     }
//     Serial.println("Success!");

//     tcaselect2(4);
//     if (!tcs.begin(0x29, &Wire1)) {
//         Serial.println("ERROR: TCS34725 NOT FOUND !!!");
//         while (1);
//     }
//     Serial.println("Success!");

//     // tcaselect2(4);
//     // if (!tcs[4].attach(Wire1))
//     //     Serial.println("ERROR: TCS34725 NOT FOUND !!!");
//     // tcs[4].integrationTime(33); // ms
//     // tcs[4].gain(TCS34725::Gain::X01);
//     // Serial.println("Success!");
// }

// void loop() 
// {
//     tcaselect2(2);

//     uint16_t r, g, b, c, colorTemp, lux;

//     tcs.getRawData(&r, &g, &b, &c);
//     // colorTemp = tcs.calculateColorTemperature(r, g, b);
//     colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
//     lux = tcs.calculateLux(r, g, b);

//     Serial.print("#2 Color11 Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
//     Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
//     Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
//     Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
//     Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
//     Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
//     Serial.println(" ");

//     tcaselect2(4);

//     tcs.getRawData(&r, &g, &b, &c);
//     // colorTemp = tcs.calculateColorTemperature(r, g, b);
//     colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
//     lux = tcs.calculateLux(r, g, b);

//     Serial.print("#4 Color11 Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
//     Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
//     Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
//     Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
//     Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
//     Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
//     Serial.println(" ");

//     // if (tcs[4].available()) {
//     //     tcaselect2(4);
//     //     TCS34725::Color color = tcs[4].color();
//     //     TCS34725::RawData rawdata = tcs[4].raw();
//     //     Serial.print("Color Temp : "); Serial.println(tcs[4].colorTemperature());
//     //     Serial.print("Lux        : "); Serial.println(tcs[4].lux());
//     //     Serial.print("R          : "); Serial.println(color.r);
//     //     Serial.print("G          : "); Serial.println(color.g);
//     //     Serial.print("B          : "); Serial.println(color.b);
//     //     Serial.print("C          : "); Serial.println(rawdata.c);
//     // }
    
// }

// void tcaselect(uint8_t i) //I2C Multiplexer: TCA9548A
// {
//     if (i >= 0 && i <= 7) {
//         Wire.beginTransmission(TCAADR);
//         Wire.write(1 << i);
//         Wire.endTransmission();
//     }
// }

// void tcaselect2(uint8_t i) //I2C Multiplexer: TCA9548A
// {
//     if (i >= 0 && i <= 7) {
//         Wire1.beginTransmission(TCAADR);
//         Wire1.write(1 << i);
//         Wire1.endTransmission();
//     }
// }