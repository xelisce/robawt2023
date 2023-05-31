#include <Arduino.h>
#include <Wire.h>
#include <TCA9548A.h>
#include <Vroom.h>
#include <ColourSensorTCS34725.h>

#define TCAADR 0x70
#define L0XADR 0x29
#define L1XADR 0x29
#define TCSADR 0x29

#define LEDPIN 3
#define SDAPIN 4
#define SCLPIN 5
#define SDA1PIN 6
#define SCL1PIN 7
#define TX0PIN 16
#define RX0PIN 17
#define PICOLEDPIN 25
#define SWTPIN 28

Motor MotorR(13, 12, 1, 0); 
Motor MotorL(10, 11, 18, 19);
Vroom Robawt(&MotorL, &MotorR);

TCA9548A BotMux(SDA1PIN, SCL1PIN, &Wire1, 400000);
ColorSensorTCS34725 colorSensor(SDA1PIN, SCL1PIN);

uint16_t r1, g1, b1, c1, colorTemp1, lux1;
uint16_t r2, g2, b2, c2, colorTemp2, lux2;
uint16_t w_1 = 12, w_2 = 10;
uint16_t b_1 = 1, b_2 = 1;
double left, right, steer;
long start_time, start_time_tcs1, end_time;

void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }

void setup()
{
    pinMode(28, INPUT);
    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);

    BotMux.begin();

    Serial.begin(9600);
    while (!Serial) delay(10);
    Serial.println("Done");

    BotMux.select(2);
    colorSensor.setWaitTime(0);  // Slow down conversion
    colorSensor.setIntegrationTime(0); // Library will limit to maximum possible time
    colorSensor.setGain(CS_GAIN_16);
    if (!colorSensor.begin()) {
        Serial.println("ERROR: TCS34725 NOT FOUND !!!");
        while (1);
    } else {
        Serial.println("Success!");
    }

    // BotMux.select(4);
    // if (!tcs2.attach(0x29, &Wire1)) {
    //     Serial.println("ERROR: TCS34725 NOT FOUND !!!");
    //     while (1);
    // } else {
    //     Serial.println("Success!");
    // }
}

long loopCount = 0;

void loop() {
  loopCount++;
  start_time = millis()
  if (colorSensor.isReady()) {
    colorSensor.clearReady();
    RGBC v = colorSensor.readRGBC();
    Serial.print("Red: ");
    Serial.print(v.r);
    Serial.print("   Green: ");
    Serial.print(v.g);
    Serial.print("   Blue: ");
    Serial.print(v.b);
    Serial.print("   Clear: ");
    Serial.print(v.c);
    Serial.print("     loop was run: ");
    Serial.print(loopCount);
    Serial.print(" times     time: ");
    Serial.println(millis() - start_time);
    loopCount = 0;
  }
}

// void loop() 
// {
//     BotMux.select(2);
//     start_time_tcs1 = millis();
//     tcs1.getRawData(&r1, &g1, &b1, &c1);
//     end_time = millis();
//     Serial.print("Time for 1 tcs: ");
//     Serial.println(end_time - start_time_tcs1);

//     // lux1 = tcs1.calculateLux(r1, g1, b1);
//     Serial.print(" | right: ");
//     // Serial.print(lux1);
//     lux1 = 0;
//     right = (double)(lux1 - b_1)/(w_1 - b_1);

//     BotMux.select(4);
//     tcs2.getRawData(&r2, &g2, &b2, &c2);
//     // lux2 = tcs2.calculateLux(r2, g2, b2);
//     Serial.print(" | left: ");
//     // Serial.print(lux2);
//     lux2 = 0;
//     left = (double)(lux2 - b_2)/(w_2 - b_2);

//     steer = (double)(left - right);
//     Serial.print(" | steer: ");
//     Serial.println(steer);

//     if (digitalRead(28)) {Robawt.setSteer(30, steer);}
//     else {Robawt.setSteer(0, 0);
//     Robawt.reset();}

//     // Serial.print("time: ");
//     // Serial.print(millis() - start_time);
// }


// //* LIBRARIES
// #include <Arduino.h>
// #include <Wire.h>
// #include "Adafruit_TCS34725.h"

// //* OBJECT INITIALISATIONS
// Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

// Adafruit_TCS34725 tcs1(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);
// Adafruit_TCS34725 tcs2(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

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