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
#include <Vroom.h>
// #include "linesense.h"

//* OBJECT INITIALISATIONS
// Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_2_4MS, TCS34725_GAIN_1X);


Adafruit_TCS34725 tcs[6] = {Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X)};

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
struct sensor
{
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;
    int hue;
    int sat;
    int val;
};

sensor sensorarray[6];
const int tcs_pins[6] = {5, 4, 2, 1, 6, 0};
const int tcs_black[6] = {0, 92, 111, 0, 0, 0};
const int tcs_white[6] = {0, 374, 494, 0, 0, 0};

sensor rgb_to_hsv(sensor *curr)
{
    // R, G, B values are divided by 255
    // to change the range from 0..255 to 0..1:
    float r = curr->r / 255.0;
    float g = curr->g / 255.0;
    float b = curr->b / 255.0;
    float cmax = max(r, max(g, b)); // maximum of r, g, b
    float cmin = min(r, min(g, b)); // minimum of r, g, b
    float diff = cmax - cmin;       // diff of cmax and cmin.
    if (cmax == cmin)
        curr->hue = 0;
    else if (cmax == r)
        curr->hue = fmod((60 * ((g - b) / diff) + 360), 360.0);
    else if (cmax == g)
        curr->hue = fmod((60 * ((b - r) / diff) + 120), 360.0);
    else if (cmax == b)
        curr->hue = fmod((60 * ((r - g) / diff) + 240), 360.0);
    // if cmax equal zero
    if (cmax == 0)
        curr->sat = 0;
    else
        curr->sat = (diff / cmax) * 100;
    // compute v
    curr->val = cmax * 100;
    return *curr;
}

Motor MotorL(12, 13, 1, 0); 
Motor MotorR(11, 10, 19, 18);
Vroom Robawt(&MotorL, &MotorR);

void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }


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

    pinMode(SWTPIN, INPUT);
    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

long start_time, end_time;
long start_loop_time;
double left, right, steer;

void loop() 
{
    start_loop_time = millis();

    for (int i=1; i<3; i++) {
        tcaselect2(tcs_pins[i]);
        start_time = millis();
        tcs[i].getRawData(&sensorarray[i].r, &sensorarray[i].g, &sensorarray[i].b, &sensorarray[i].c);
        end_time = millis();
        Serial.print("Sensor "); Serial.print(i); Serial.print("    ");
        Serial.print("R: "); Serial.print(sensorarray[i].r); Serial.print(" ");
        Serial.print("G: "); Serial.print(sensorarray[i].g); Serial.print(" ");
        Serial.print("B: "); Serial.print(sensorarray[i].b); Serial.print(" ");
        Serial.print("C: "); Serial.print(sensorarray[i].c); Serial.print(" ");
        sensorarray[i] = rgb_to_hsv(&sensorarray[i]);
        Serial.print("R: "); Serial.print(sensorarray[i].r); Serial.print(" ");
        Serial.print("G: "); Serial.print(sensorarray[i].g); Serial.print(" ");
        Serial.print("B: "); Serial.print(sensorarray[i].b); Serial.print(" ");
        Serial.print("C: "); Serial.print(sensorarray[i].c); Serial.print(" ");
        Serial.print("H: "); Serial.print(sensorarray[i].hue, DEC); Serial.print(" ");
        Serial.print("S: "); Serial.print(sensorarray[i].sat, DEC); Serial.print(" ");
        Serial.print("V: "); Serial.print(sensorarray[i].val, DEC); Serial.print(" ");
        Serial.print("Time: "); Serial.print(end_time - start_time, DEC); Serial.print(" ");
        Serial.println();
    }
    
    left = (double)(sensorarray[1].val - tcs_black[1])/(double)(tcs_white[1] - tcs_black[1]);
    right = (double)(sensorarray[2].val - tcs_black[2])/(double)(tcs_white[2] - tcs_black[2]);
    steer = left - right;
    Serial.print("left: "); Serial.print(left); Serial.print(" ");
    Serial.print("right: "); Serial.print(right); Serial.print(" ");
    Serial.print("steer: "); Serial.print(steer); Serial.print("  ");
    Serial.print("time: "); Serial.print(millis() - start_loop_time); Serial.println("  ");

    if (digitalRead(SWTPIN)) {
        Robawt.setSteer(30, steer);
    } else {
        Robawt.setSteer(0, 0);
        Robawt.reset();
    }
    // double hue, sat, value;
    // long start_2;
    // start_2 = millis();
    // Serial.print("Hue values: "); 
    // hsv.calibrateSensors();
    // Serial.print("library time: "); Serial.println(millis() - start_2);

    // long start, end;
    // struct hsv hsvm;
    
    // uint16_t r, g, b, c;
    // for (int i=0; i<6;i++){
    //     start = millis();
    //     tcaselect2(tcs_pins[i]);
    //     tcs[i].getRawData(&r, &g, &b, &c);
    //     end = millis();
    //     Serial.print("TCS no: "); Serial.print(tcs_pins[i]); Serial.print(" | ");
    //     Serial.print("R: "); Serial.print(r, DEC); Serial.print(" ");
    //     Serial.print("G: "); Serial.print(g, DEC); Serial.print(" ");
    //     Serial.print("B: "); Serial.print(b, DEC); Serial.print(" ");
    //     Serial.print("C: "); Serial.print(c, DEC); Serial.print(" ");
    //     Serial.print("| "); Serial.println(end - start);

    //     hsvm = rgb_to_hsv(r, g, b);
    //     Serial.print("Hue: "); Serial.print(hsvm.hue);
    //     Serial.print(" Sat: ");Serial.print(hsvm.sat);
    //     Serial.print(" Val: ");Serial.print(hsvm.val);
    //     Serial.println();
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



/* NEW 

white
Sensor 0    R: 1570 G: 2071 B: 1883 C: 5506 R: 1570 G: 2071 B: 1883 C: 5506 H: 157 S: 24 V: 812 Time: 1 
Sensor 1    R: 680 G: 955 B: 922 C: 2587 R: 680 G: 955 B: 922 C: 2587 H: 172 S: 28 V: 374 Time: 1 
Sensor 2    R: 1063 G: 1262 B: 1209 C: 3601 R: 1063 G: 1262 B: 1209 C: 3601 H: 164 S: 15 V: 494 Time: 0 
Sensor 3    R: 591 G: 874 B: 824 C: 2326 R: 591 G: 874 B: 824 C: 2326 H: 169 S: 32 V: 342 Time: 1 
Sensor 4    R: 1090 G: 1752 B: 1658 C: 4458 R: 1090 G: 1752 B: 1658 C: 4458 H: 171 S: 37 V: 687 Time: 1 
Sensor 5    R: 910 G: 1062 B: 1046 C: 3129 R: 910 G: 1062 B: 1046 C: 3129 H: 173 S: 14 V: 416 Time: 0 

black
Sensor 0    R: 193 G: 285 B: 271 C: 759 R: 193 G: 285 B: 271 C: 759 H: 170 S: 32 V: 111 Time: 0 
Sensor 1    R: 155 G: 236 B: 234 C: 642 R: 155 G: 236 B: 234 C: 642 H: 178 S: 34 V: 92 Time: 1 
Sensor 2    R: 210 G: 284 B: 284 C: 804 R: 210 G: 284 B: 284 C: 804 H: 180 S: 26 V: 111 Time: 0 
Sensor 3    R: 141 G: 229 B: 224 C: 606 R: 141 G: 229 B: 224 C: 606 H: 176 S: 38 V: 89 Time: 1 
Sensor 4    R: 107 G: 185 B: 192 C: 484 R: 107 G: 185 B: 192 C: 484 H: 184 S: 44 V: 75 Time: 0 
Sensor 5    R: 144 G: 193 B: 204 C: 565 R: 144 G: 193 B: 204 C: 565 H: 191 S: 29 V: 80 Time: 1 


*/