/*#include <Arduino.h>
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

Motor MotorR(12, 13, 1, 0); 
Motor MotorL(11, 10, 19, 18);
Vroom Robawt(&MotorL, &MotorR);

void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }

// TCA9548A BotMux(SDA1PIN, SCL1PIN, &Wire1, 400000);
// ColorSensorTCS34725 colorSensor(SDA1PIN, SCL1PIN);

uint16_t r1, g1, b1, c1, colorTemp1, lux1;
uint16_t r2, g2, b2, c2, colorTemp2, lux2;
uint16_t w_1 = 12, w_2 = 10;
uint16_t b_1 = 1, b_2 = 1;
double left, right, steer;
long start_time, start_time_tcs1, end_time;


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
  start_time = millis();
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

*/

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


//* LIBRARIES
#include <Arduino.h>
#include <Wire.h>
#include "Adafruit_TCS34725.h"
// #include "linesense.h"

//* OBJECT INITIALISATIONS
// Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);

Adafruit_TCS34725 tcs1(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);
Adafruit_TCS34725 tcs2(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);

Adafruit_TCS34725 tcs[6] = {Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X)};

// LineSense array;

const int tcs_pins[6] = {0,1,2,4,5,6};

//* ADDRESSES
#define TCAADR 0x70
#define L0XADR 0x29
#define L1XADR 0x29
#define TCSADR 0x29

//* PINS
#define LEDPIN 3
#define SDAPIN 4
#define SCLPIN 5
#define SDA1PIN 6
#define SCL1PIN 7
#define TX0PIN 16
#define RX0PIN 17
#define PICOLEDPIN 25
#define SWTPIN 28
struct hsv
{
    int hue;
    int sat;
    int val;
};

struct hsv rgb_to_hsv(float r, float g, float b)
{
    // R, G, B values are divided by 255
    // to change the range from 0..255 to 0..1:
    float h, s, v;
    r /= 255.0;
    g /= 255.0;
    b /= 255.0;
    float cmax = max(r, max(g, b)); // maximum of r, g, b
    float cmin = min(r, min(g, b)); // minimum of r, g, b
    float diff = cmax - cmin;       // diff of cmax and cmin.
    if (cmax == cmin)
        h = 0;
    else if (cmax == r)
        h = fmod((60 * ((g - b) / diff) + 360), 360.0);
    else if (cmax == g)
        h = fmod((60 * ((b - r) / diff) + 120), 360.0);
    else if (cmax == b)
        h = fmod((60 * ((r - g) / diff) + 240), 360.0);
    // if cmax equal zero
    if (cmax == 0)
        s = 0;
    else
        s = (diff / cmax) * 100;
    // compute v
    v = cmax * 100;

    struct hsv curr;
    curr.hue = h;
    curr.sat = s;
    curr.val = v;
    return curr;
}

void setup() 
{
    Serial.begin(9600);
    while (!Serial) delay(10);
    Serial.println("Serial intialised");
    
    Wire1.setSDA(SDA1PIN);
    Wire1.setSCL(SCL1PIN);
    Wire1.begin();
    Wire1.setClock(400000);
    Serial.println("TCA initialised");

    for (int i=0;i<6;i++){
        tcaselect2(tcs_pins[i]);
        if (!tcs[i].begin(0x29, &Wire1)) {
            Serial.println("ERROR: TCS34725 1 NOT FOUND !!!");
            while (1);
        }
        Serial.println("Success!");

    }

}

void loop() 
{

    // tcaselect2(4);
    
    // uint16_t r, g, b, c, colorTemp, lux;
    // tcs1.getRawData(&r, &g, &b, &c);
    // Serial.print("Time for 1 tcs: "); Serial.println(millis() - start); 
    
    // // colorTemp = tcs.calculateColorTemperature(r, g, b);
    // // colorTemp = tcs1.calculateColorTemperature_dn40(r, g, b, c);
    // // lux = tcs1.calculateLux(r, g, b);

    // // Serial.print("#2 Color11 Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
    // // Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
    // Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    // Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    // Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    // Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
    // Serial.println(" ");

    
    // double hue, sat, value;
    // long start_2;
    // start_2 = millis();
    // Serial.print("Hue values: "); 
    // hsv.calibrateSensors();
    // Serial.print("library time: "); Serial.println(millis() - start_2);

    long start, end;
    struct hsv hsvm;
    
    uint16_t r, g, b, c;
    for (int i=0; i<6;i++){
        start = millis();
        tcaselect2(tcs_pins[i]);
        tcs[i].getRawData(&r, &g, &b, &c);
        end = millis();
        Serial.print("TCS no: "); Serial.print(tcs_pins[i]); Serial.print(" | ");
        Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
        Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
        Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
        Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
        Serial.print("| "); Serial.println(end - start);

        hsvm = rgb_to_hsv(r, g, b);
        Serial.print("Hue: "); Serial.print(hsvm.hue);
        Serial.print(" Sat: ");Serial.print(hsvm.sat);
        Serial.print(" Val: ");Serial.print(hsvm.val);
        Serial.println();
    }
    

    // tcaselect2(4);

    // tcs2.getRawData(&r, &g, &b, &c);
    // // colorTemp = tcs.calculateColorTemperature(r, g, b);
    // colorTemp = tcs2.calculateColorTemperature_dn40(r, g, b, c);
    // lux = tcs2.calculateLux(r, g, b);

    // Serial.print("#4 Color11 Temp: "); Serial.print(colorTemp, DEC); Serial.print(" K - ");
    // Serial.print("Lux: "); Serial.print(lux, DEC); Serial.print(" - ");
    // Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    // Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    // Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    // Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
    // Serial.println(" ");
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



/* Readings */
/* 
TCS no: 0 | R: 145 G: 190 B: 199 C: 554 | 1
Hue: 190 Sat: 27 Val: 78
TCS no: 1 | R: 513 G: 748 B: 701 C: 1978 | 1
Hue: 168 Sat: 31 Val: 293
TCS no: 2 | R: 801 G: 933 B: 898 C: 2673 | 1
Hue: 164 Sat: 14 Val: 365
TCS no: 4 | R: 195 G: 329 B: 274 C: 801 | 0
Hue: 155 Sat: 40 Val: 129
TCS no: 5 | R: 320 G: 554 B: 443 C: 1299 | 1
Hue: 151 Sat: 42 Val: 217
TCS no: 6 | R: 105 G: 223 B: 183 C: 509 | 0
Hue: 159 Sat: 52 Val: 87




Green: 
TCS no: 5 | R: 190 G: 409 B: 293 C: 890 | 1
Hue: 148 Sat: 53 Val: 160
TCS no: 4 | R: 175 G: 356 B: 284 C: 826 | 1
Hue: 156 Sat: 50 Val: 139
TCS no: 2 | R: 262 G: 498 B: 383 C: 1178 | 1
Hue: 150 Sat: 47 Val: 195
TCS no: 1 | R: 212 G: 432 B: 364 C: 1036 | 1
Hue: 161 Sat: 50 Val: 169
TCS no: 0 | R: 130 G: 219 B: 173 C: 545 | 1
Hue: 148 Sat: 40 Val: 85
TCS no: 6 | R: 163 G: 413 B: 300 C: 870 | 0
Hue: 152 Sat: 60 Val: 161


Front 4 on black:
TCS no: 1 | R: 262 G: 385 B: 370 C: 1028 | 1
Hue: 172 Sat: 31 Val: 150
TCS no: 2 | R: 430 G: 514 B: 507 C: 1480 | 0
Hue: 175 Sat: 16 Val: 201
TCS no: 4 | R: 305 G: 417 B: 408 C: 1140 | 1
Hue: 175 Sat: 26 Val: 163
TCS no: 5 | R: 444 G: 583 B: 539 C: 1554 | 0
Hue: 161 Sat: 23 Val: 228

Back 2 on black:
TCS no: 0 | R: 163 G: 212 B: 228 C: 641 | 0
Hue: 194 Sat: 28 Val: 89
TCS no: 6 | R: 133 G: 232 B: 241 C: 614 | 1
Hue: 185 Sat: 44 Val: 94

TCS no: 0 | R: 154 G: 199 B: 213 C: 600 | 1
Hue: 194 Sat: 27 Val: 83
TCS no: 6 | R: 110 G: 180 B: 188 C: 476 | 1
Hue: 186 Sat: 41 Val: 73


All white:
TCS no: 0 | R: 896 G: 1057 B: 1040 C: 3132 | 1
Hue: 173 Sat: 15 Val: 414
TCS no: 1 | R: 565 G: 833 B: 786 C: 2211 | 0
Hue: 169 Sat: 32 Val: 326
TCS no: 2 | R: 919 G: 1103 B: 1064 C: 3152 | 1
Hue: 167 Sat: 16 Val: 432
TCS no: 4 | R: 603 G: 834 B: 806 C: 2273 | 0
Hue: 172 Sat: 27 Val: 327
TCS no: 5 | R: 836 G: 1115 B: 1026 C: 2971 | 1
Hue: 160 Sat: 25 Val: 437
TCS no: 6 | R: 763 G: 1234 B: 1179 C: 3154 | 0
Hue: 172 Sat: 38 Val: 483



TCS no: 0 | R: 885 G: 1044 B: 1029 C: 3096 | 1
Hue: 174 Sat: 15 Val: 409
TCS no: 1 | R: 582 G: 861 B: 814 C: 2286 | 0
Hue: 169 Sat: 32 Val: 337
TCS no: 2 | R: 1008 G: 1201 B: 1157 C: 3433 | 1
Hue: 166 Sat: 16 Val: 470
TCS no: 4 | R: 631 G: 885 B: 857 C: 2404 | 1
Hue: 173 Sat: 28 Val: 347
TCS no: 5 | R: 1059 G: 1393 B: 1280 C: 3716 | 1
Hue: 159 Sat: 23 Val: 546
TCS no: 6 | R: 914 G: 1471 B: 1407 C: 3764 | 1
Hue: 173 Sat: 37 Val: 576

*/
