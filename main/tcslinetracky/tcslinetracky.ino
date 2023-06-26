#include <Arduino.h>
#include <Adafruit_TCS34725.h>
#include <Wire.h>
#include <Vroom.h>

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

Motor MotorL(12, 13, 0, 1); 
Motor MotorR(11, 10, 19, 18);
Vroom Robawt(&MotorL, &MotorR);
void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }

int pinL1 = 12, pinL2 = 13;
int pinR1 = 11, pinR2 = 10;

long beforeTCSLoopTimeMicros, afterTCSLoopTimeMicros;

//* TCS SETUP
const int tcsNum = 6;
Adafruit_TCS34725 tcs[6] = {Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X)};
struct tcsSensor
{
    uint16_t r;
    uint16_t g;
    uint16_t b;
    uint16_t c;
    int hue;
    int sat;
    int val;
    bool green = false;
    bool red = false;
    bool black = false;
    bool silver = false;
};
struct tcsSensor tcsSensors[tcsNum];
const int tcs_pins[tcsNum] = {5, 4, 2, 1, 6, 0}; //from left to right, then second row with robot facing forward
const int tcs_black[tcsNum] = {0, 104, 124, 0, 0, 0};
const int tcs_white[tcsNum] = {0, 331, 410, 0, 0, 0};
const int tcs_lgreen[tcsNum] = {0, 0, 0, 0, 0, 0};
const int tcs_ugreen[tcsNum] = {0, 0, 0, 0, 0, 0};
const int tcs_lred1[tcsNum] = {0, 0, 0, 0, 0, 0};
const int tcs_ured1[tcsNum] = {0, 0, 0, 0, 0, 0};
const int tcs_lred2[tcsNum] = {0, 0, 0, 0, 0, 0};
const int tcs_ured2[tcsNum] = {0, 0, 0, 0, 0, 0};


long startLoopMicros, endLoopMicros;

void setup() 
{
    pinMode(pinL1, OUTPUT);
    pinMode(pinL2, OUTPUT);
    pinMode(pinR1, OUTPUT);
    pinMode(pinR2, OUTPUT);

    Wire1.setSDA(SDA1PIN);
    Wire1.setSCL(SCL1PIN);
    Wire1.begin();
    Wire1.setClock(400000);
    Serial.println("Bottom multiplexer initialised");

    Serial.begin(115200);
    pinMode(28, INPUT_PULLDOWN);

    for (int i = 1; i < 3; i++) {
        tcaselect2(tcs_pins[i]);
        while (!tcs[i].begin(TCSADR, &Wire1)) { Serial.println("ERROR: TCS34725 No. "); Serial.print(i); Serial.println(" NOT FOUND!"); }
    }
    Serial.println("TCS sensors initialised");

    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
    Serial.println("Encoders initialised");

}

void loop()
{
    startLoopMicros = micros();
    Serial.print("switch");
    Serial.println(digitalRead(28));
    if (digitalRead(28)) { 
        for (int i = 1; i < 3; i++) { //^ only using 2 to linetrack right now
            tcaselect2(tcs_pins[i]);
            #if debugLoopTime
            beforeEachTCSLoopTimeMicros[i] = micros();
            #endif
            tcs[i].getRawData(&tcsSensors[i].r, &tcsSensors[i].g, &tcsSensors[i].b, &tcsSensors[i].c);
            rgb_to_hsv(i);
            #if debugLoopTime
            afterEachTCSLoopTimeMicros[i] = micros();
            #endif
        }

        bool left = isBlack(1);
        bool right = isBlack(2);
        Serial.print(left); Serial.print("\t"); Serial.print(right);

        if (left) {
            if (right) {
                Robawt.setSteer(40, 0);
            } else {
                Robawt.setSteer(40, -1);
            }
        } else  {
            if (right) {
                Robawt.setSteer(40, 1);
            } else {
                Robawt.setSteer(40,0);
            }
        }
    } else {
        Robawt.stop();
        for (int i = 0; i < tcsNum; i++) {
        Serial.print("Sensor "); Serial.print(i); Serial.print("    ");
        Serial.print("R: "); Serial.print(tcsSensors[i].r); Serial.print(" ");
        Serial.print("G: "); Serial.print(tcsSensors[i].g); Serial.print(" ");
        Serial.print("B: "); Serial.print(tcsSensors[i].b); Serial.print(" ");
        Serial.print("C: "); Serial.print(tcsSensors[i].c); Serial.print(" ");
        Serial.print("H: "); Serial.print(tcsSensors[i].hue, DEC); Serial.print(" ");
        Serial.print("S: "); Serial.print(tcsSensors[i].sat, DEC); Serial.print(" ");
        Serial.print("V: "); Serial.print(tcsSensors[i].val, DEC); Serial.print(" ");
        // Serial.print("Time: "); Serial.print(afterEachTCSLoopTimeMicros[i] - beforeEachTCSLoopTimeMicros[i]);
        Serial.println();
    }
    }
    endLoopMicros = micros();
    Serial.print("\t"); Serial.println(endLoopMicros - startLoopMicros);
}

void rgb_to_hsv(int i)
{
    // R, G, B values are divided by 255
    // to change the range from 0..255 to 0..1:
    struct tcsSensor *currentTCS = &(tcsSensors[i]);
    float r = currentTCS->r / 255.0;
    float g = currentTCS->g / 255.0;
    float b = currentTCS->b / 255.0;
    float cmax = max(r, max(g, b)); // maximum of r, g, b
    float cmin = min(r, min(g, b)); // minimum of r, g, b
    float diff = cmax - cmin;       // diff of cmax and cmin.
    if (cmax == cmin)
        currentTCS->hue = 0;
    else if (cmax == r)
        currentTCS->hue = fmod((60 * ((g - b) / diff) + 360), 360.0);
    else if (cmax == g)
        currentTCS->hue = fmod((60 * ((b - r) / diff) + 120), 360.0);
    else if (cmax == b)
        currentTCS->hue = fmod((60 * ((r - g) / diff) + 240), 360.0);
    // if cmax equal zero
    if (cmax == 0)
        currentTCS->sat = 0;
    else
        currentTCS->sat = (diff / cmax) * 100;
    // compute v
    currentTCS->val = cmax * 100;
}

double grayPercent(int i) 
{
    return (double)(tcsSensors[i].val - tcs_black[i])/(double)(tcs_white[i] - tcs_black[i]);
}

bool isBlack(int i)
{
    if (!(tcsSensors[i].green) && !(tcsSensors[i].red) && grayPercent(i) < 0.5) { return true; }
    else { return false; }
}


void tcaselect2(uint8_t i) //I2C Bottom Multiplexer: TCA9548A
{
    if (i > 0 && i < 7) {
        Wire1.beginTransmission(TCAADR);
        Wire1.write(1 << i);
        Wire1.endTransmission();
    }
}