//* LIBRARIES
#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <Vroom.h>
#include <Adafruit_TCS34725.h>
#include <VL53L0X.h>
#include <VL53L1X.h>
#include <myEnums.h>

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

//* SETTINGS
#define waitForSerial 0
#define debugLoopTime 1
#define debugTCSReadings 1
#define debugLED 0

//* ------------------------------------------- OBJECT INITIALISATIONS -------------------------------------------

//* DRIVEBASE SETUP
Motor MotorL(12, 13, 1, 0); 
Motor MotorR(11, 10, 19, 18);
Vroom Robawt(&MotorL, &MotorR);
void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }

//* SERVOS SETUP
const int servosNum = 6;
Servo servos[servosNum];
const int servos_pins[servosNum] = {27, 26, 22, 21, 20, 2};
double servos_angle[servosNum] = {0, 0, 180, 0, 130, 177}; //basic states initialised
const double servos_max_angle[servosNum] = {180, 180, 300, 300, 300, 180};
bool servos_change = true;
namespace Servos {
    enum Servos { DEAD, ALIVE, SORT, LEFT, RIGHT, ARM};
}

//* LIDARS SETUP
const int defaultLidarReading = 200;
//^ VL53L0X
const int L0XNum = 4;
VL53L0X lidarsl0x[L0XNum];
int l0x_readings[L0XNum] = {defaultLidarReading, defaultLidarReading, defaultLidarReading, defaultLidarReading};
const int l0x_pins[L0XNum] = {1, 5, 6, 0};
String l0x_labels[L0XNum] = {"FRONT: ", "FRONT LEFT: ", "LEFT: ", "RIGHT: "}; //for print debugging
namespace L0X {
    enum L0X { FRONT, FRONT_LEFT, LEFT, RIGHT };
}
//^ VL53L1X
const int L1XNum = 1;
VL53L1X lidarsl1x[L1XNum];
int l1x_readings[L1XNum] = {defaultLidarReading};
const int l1x_pins[L1XNum] = {4};
String l1x_labels[L1XNum] = {"FRONT BOTTOM: "}; //for print debugging
namespace L1X {
    enum L1X { FRONT_BOTTOM };
}

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
};
struct tcsSensor tcsSensors[tcsNum];
const int tcs_pins[tcsNum] = {5, 4, 2, 1, 6, 0}; //from left to right, then second row with robot facing forward
const int tcs_black[tcsNum] = {0, 104, 124, 0, 0, 0};
const int tcs_white[tcsNum] = {0, 331, 410, 0, 0, 0};

//* ------------------------------------------- USER-DEFINED VARIABLES -------------------------------------------

//^ SWITCHES
commandType cmd = LT;
currType curr = LINETRACK;

//^ DEBUG
long beforeEntireLoopTimeMicros, afterEntireLoopTimeMicros;
long beforeLidarLoopTimeMicros, afterLidarLoopTimeMicros;
long beforeTCSLoopTimeMicros, afterTCSLoopTimeMicros;
long beforeEachTCSLoopTimeMicros[tcsNum], afterEachTCSLoopTimeMicros[tcsNum];
double debugLDist, debugRDist;
bool ledOn = false;

//^ LOGIC TIMINGS
long lastSerialPiSendMillis = millis();

//^ ESSENTIALS
double steer = 0, rpm = 30;
double lt_rpm = 30;

//^ LINETRACK VARIABLES
double left = 1, right = 1;

//^ FORCED VARIABLES
bool forcedTurnAlrInit = false;
currType postForcedDistCase = LINETRACK;
double startForcedDistL, startForcedDistR, currForcedDist;
double wantedForcedDist = 0;
double forcedDirection = 0;

//* ------------------------------------------ START SETUP -------------------------------------------

void setup() 
{
    Serial.println("----- void setup begin -----");

    //^ PIO
    pinMode(SWTPIN, INPUT);
    pinMode(PICOLEDPIN, OUTPUT);
    pinMode(LEDPIN, OUTPUT);
    digitalWrite(LEDPIN, LOW);
    digitalWrite(PICOLEDPIN, LOW);
    Serial.println("PIO pins initialised");

    //^ USB SERIAL COMMS
    Serial.begin(9600);
    #if waitForSerial
    while (!Serial) delay(10);
    #endif
    Serial.println("USB serial initialised");

    //^ PI SERIAL COMMS
    Serial1.setRX(RX0PIN); 
    Serial1.setTX(TX0PIN);
    Serial1.begin(9600);
    while (!Serial1) delay(10); 
    Serial.println("Pi serial initialised");

    //^ MULTIPLEXERS
    Wire.setSDA(SDAPIN);
    Wire.setSCL(SCLPIN);
    Wire.begin();
    Wire.setClock(400000);
    Serial.println("Top multiplexer initialised");
    Wire1.setSDA(SDA1PIN);
    Wire1.setSCL(SCL1PIN);
    Wire1.begin();
    Wire1.setClock(400000);
    Serial.println("Bottom multiplexer initialised");

    //^ LIDARS
    for (int i = 0; i < L0XNum; i++) {
        tcaselect(l0x_pins[i]);
        Serial.println(i);
        lidarsl0x[i].setTimeout(500);
        while (!lidarsl0x[i].init()) { Serial.print("ERROR: "); Serial.print(l0x_labels[i]); Serial.print("at pin "); Serial.print(l0x_pins[i]); Serial.println(" - L0X failed to initialise"); }
        lidarsl0x[i].startContinuous();
    }
    for (int i = 0; i < L1XNum; i++) {
        tcaselect(l1x_pins[i]);
        Serial.println(i);
        lidarsl1x[i].setTimeout(500);
        while (!lidarsl1x[i].init()) { Serial.print("ERROR: "); Serial.print(l1x_labels[i]); Serial.print("at pin "); Serial.print(l1x_pins[i]); Serial.print(" - L1X failed to initialise"); }
        lidarsl1x[i].setDistanceMode(VL53L1X::Medium);
        lidarsl1x[i].startContinuous(20);
    }
    Serial.println("Lidars initialised");

    //^ SERVOS
    for (int i = 0; i < servosNum; i++) {
        servos[i].attach(servos_pins[i], 500, 2500); // (pin, min, max)
        servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
    }
    Serial.println("Servos initialised");

    //^ TCS
    // follows the numbering 
    // | 0 1 2 3 4 |
    // |    5 6    |
    for (int i = 0; i < tcsNum; i++) {
        tcaselect2(tcs_pins[i]);
        while (!tcs[i].begin(TCSADR, &Wire1)) { Serial.println("ERROR: TCS34725 No. "); Serial.print(i); Serial.println(" NOT FOUND!"); }
    }
    Serial.println("TCS sensors initialised");

    //^ MOTOR ENCODERS
    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
    Serial.println("Encoders initialised");

    Serial.println("----- void setup complete -----");
}

//* ------------------------------------------- START LOOP -------------------------------------------

void loop()
{
    #if debugLoopTime
    beforeEntireLoopTimeMicros = micros();
    #endif
    #if debugWithLED
    if (ledOn) digitalWrite(PICOLEDPIN, HIGH);
    else digitalWrite(PICOLEDPIN, LOW);
    ledOn = false;
    #endif

    //* SERVOS UPDATES
    if (servos_change) {
        for (int i = 0; i < servosNum; i++) {
        servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
        }
        servos_change = false;
    }

    //* LIDAR READINGS
    #if debugLoopTime
    beforeLidarLoopTimeMicros = micros();
    #endif
    for (int i = 0; i < L0XNum; i++) 
    {
        tcaselect(l0x_pins[i]);
        if (lidarsl0x[i].available()) { l0x_readings[i] = lidarsl0x[i].readRangeMillimeters(); }
    }
    for (int i = 0; i < L1XNum; i++) 
    {
        tcaselect(l1x_pins[i]);
        if (lidarsl1x[i].dataReady()) { l1x_readings[i] = lidarsl1x[i].read(false); }
    }
    #if debugLoopTime
    afterLidarLoopTimeMicros = micros();
    #endif

    //* TCS READINGS
    #if debugLoopTime
    beforeTCSLoopTimeMicros = micros();
    #endif
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
    #if debugLoopTime
    afterTCSLoopTimeMicros = micros();
    #endif

    if (digitalRead(SWTPIN))
    {
        //* ------------------------------------------- CURRENT SITUATION HANDLED -------------------------------------------

        switch (curr)
        {
            case LINETRACK:
                left = (double)(tcsSensors[1].val - tcs_black[1])/(double)(tcs_white[1] - tcs_black[1]);
                right = (double)(tcsSensors[2].val - tcs_black[2])/(double)(tcs_white[2] - tcs_black[2]);
                steer = (left-right)>0 ? pow(left - right, 0.5) : -pow(left - right, 0.5);
                Robawt.setSteer(lt_rpm, steer);
                break;

            case LEFTGREEN:
                turnAngle(-0.8, 75, STOP);
                break;

            case RIGHTGREEN:
                turnAngle(0.8, 75, STOP);
                break;

            case DOUBLEGREEN:
                turnAngle(1, 180, STOP);
                break;

            case RED:
                curr = STOP;
                break;

            //* ------------------------------------------- FORCED MOVEMENTS -------------------------------------------
                
            case STOP:
                Robawt.setSteer(0, 0);
                Robawt.resetPID();
                break;

            case MOVEDIST:
                currForcedDist = getRotated(startForcedDistL, startForcedDistR);
                if (currForcedDist >= wantedForcedDist) { curr = postForcedDistCase; }
                else { Robawt.setSteer(forcedDirection*30, 0); }
                break;

            case TURNANGLE:
                currForcedDist = getRotated(startForcedDistL, startForcedDistR);
                if (currForcedDist >= wantedForcedDist) { curr = postForcedDistCase; }
                else { Robawt.setSteer(30, forcedDirection); }
                break;

            default:
                #if debugWithLED
                ledOn = true;
                #endif
                Serial.println("ERROR: variable curr undefined, default case triggered");
                break;
        }

    } else {
        //* ------------------------------------------- SWITCH OFF -------------------------------------------

        Robawt.setSteer(0, 0);
        Robawt.resetPID();
        curr = LINETRACK;
    }

    //* ------------------------------------------- DEBUG PRINTS -------------------------------------------

    #if debugLoopTime
    afterEntireLoopTimeMicros = micros();
    Serial.print("Time   ");
    Serial.print("Loop: "); Serial.print(afterEntireLoopTimeMicros - beforeEntireLoopTimeMicros); Serial.print(" ");
    Serial.print("TCS: "); Serial.print(afterTCSLoopTimeMicros - beforeTCSLoopTimeMicros); Serial.print(" ");
    Serial.print("LiDAR: "); Serial.print(afterLidarLoopTimeMicros - beforeLidarLoopTimeMicros); Serial.print(" ");
    Serial.println();
    #endif

    #if debugDist
    Serial.print("L: "); Serial.print(debugLDist); Serial.print(" ");
    Serial.print("R: "); Serial.print(debugRDist); Serial.print(" ");
    #endif

    #if debugTCSReadings
    for (int i = 0; i < tcsNum; i++) {
        Serial.print("Sensor "); Serial.print(i); Serial.print("    ");
        Serial.print("R: "); Serial.print(tcsSensors[i].r); Serial.print(" ");
        Serial.print("G: "); Serial.print(tcsSensors[i].g); Serial.print(" ");
        Serial.print("B: "); Serial.print(tcsSensors[i].b); Serial.print(" ");
        Serial.print("C: "); Serial.print(tcsSensors[i].c); Serial.print(" ");
        Serial.print("H: "); Serial.print(tcsSensors[i].hue, DEC); Serial.print(" ");
        Serial.print("S: "); Serial.print(tcsSensors[i].sat, DEC); Serial.print(" ");
        Serial.print("V: "); Serial.print(tcsSensors[i].val, DEC); Serial.print(" ");
        Serial.print("Time: "); Serial.print(afterEachTCSLoopTimeMicros[i] - beforeEachTCSLoopTimeMicros[i]);
        Serial.println();
    }
    #endif
}

//* ------------------------------------------- USER DEFINED FUNCTIONS -------------------------------------------

//* COMMUNICATIONS

void send_pi(int i) //Pico to pi serial
{
    if (millis() - lastSerialPiSendMillis > 100) { //to not spam the pi with messages
        Serial1.print(i);
        lastSerialPiSendMillis = millis();
    }
}

void tcaselect(uint8_t i) //I2C Top Multiplexer: TCA9548A
{
    if (i > 0 && i < 7) {
        Wire.beginTransmission(TCAADR);
        Wire.write(1 << i);
        Wire.endTransmission();
    }
}

void tcaselect2(uint8_t i) //I2C Bottom Multiplexer: TCA9548A
{
    if (i > 0 && i < 7) {
        Wire1.beginTransmission(TCAADR);
        Wire1.write(1 << i);
        Wire1.endTransmission();
    }
}

//* CLAW & SERVO FUNCTIONS

int pwmangle(double angle, double max_angle) //Servo PWM
{
    return (int)(angle/max_angle * 2000 + 500);
}

void claw_open() {
    servos_angle[Servos::RIGHT] = 130; 
    servos_angle[Servos::LEFT] = 0;
    servos_change = true;
}

void claw_close() { //ball
    servos_angle[Servos::RIGHT] = 25;
    servos_angle[Servos::LEFT] = 105;
    servos_change = true;
}

void claw_close_cube() {
    servos_angle[Servos::RIGHT] = 10;
    servos_angle[Servos::LEFT] = 120;
    servos_change = true;
}

void claw_halfclose() {
    servos_angle[Servos::RIGHT] = 95;
    servos_angle[Servos::LEFT] = 35;
    servos_change = true;
}

void claw_up() {
    servos_angle[Servos::ARM] = 0;
    servos_change = true;
}

void claw_service_up() {
    servos_angle[Servos::ARM] = 140;
    servos_change = true;
}

void claw_down() {
    servos_angle[Servos::ARM] = 177;
    servos_change = true;
}

void alive_up() {
    servos_angle[Servos::ALIVE] = 10;
    servos_change = true;
}

void alive_down() {
    servos_angle[Servos::ALIVE] = 100;
    servos_change = true;
}

void dead_up() { 
    servos_angle[Servos::DEAD] = 65;
    servos_change = true;
}

void dead_down() {
    servos_angle[Servos::DEAD] = 10;
    servos_change = true;
}

void sort_alive() {
    servos_angle[Servos::SORT] = 130;
    servos_change = true;
}

void sort_neutral() {
    servos_angle[Servos::SORT] = 90;
    servos_change = true;
}

void sort_dead() {
    servos_angle[Servos::SORT] = 50;
    servos_change = true;
}

//* MISCELLANEOUS

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

double pickMotorDist(double rot) { 
    if (rot > 0) { return MotorL.getDist(); }
    else { return MotorR.getDist(); }
}

double getRotated(double leftEnc, double rightEnc) {
    return ((abs(pickMotorDist(1)-leftEnc) + abs(pickMotorDist(-1)-rightEnc)) / 2);
}

double constraint(double val, double minVal, double maxVal)
{
    return min(max(val, minVal), maxVal);
}

void turnAngle(double rot, double angleOfTurn, enum currType postCase) //in degrees
{
    //only support: 0.5 OR 1 rot, speed 30 rpm, < 180 degrees. note: angle is always positive
    forcedDirection = rot;
    startForcedDistL = MotorL.getDist();
    startForcedDistR = MotorR.getDist();
    curr = TURNANGLE;
    if (abs(rot) == 1) {
        wantedForcedDist = angleOfTurn / 5.95;
        wantedForcedDist += constraint((angleOfTurn - 90)/60, -0.5, 0.5); //to account for discrepancies in smaller angle turns: decel and accel requires a tighter distance to work with, as well as larger distance means higher uncertainty between the unsynced steering
    } else { //for one wheel turns
        wantedForcedDist = angleOfTurn / 5.75;
        wantedForcedDist += constraint(((angleOfTurn - 90)/90), -0.5, 0.5);
    }
    postForcedDistCase = postCase;
}

void moveDist(double dir, double distOfMove, enum currType postCase) //in cm
{
    //only support: speed 30 rpm. note: put .0 if whole number, or problems will happen
    forcedDirection = dir; //1 or -1 for forward and backward respectively
    startForcedDistL = MotorL.getDist();
    startForcedDistR = MotorR.getDist();
    curr = MOVEDIST;
    wantedForcedDist = distOfMove * 1.47;
    postForcedDistCase = postCase;
}