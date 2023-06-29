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
#define debugTCSReadings 0
#define debugLidarReadings 1
#define debugWithLED 1
#define debugSerial 0
#define debugState 1

#define debugAlignSweep 1
#define debugLinegapSweep 0

//* FUNCTION HEADERS
void turnAngle(double rot, double angleOfTurn, double speedOfTurn, enum currType postState, enum currType afterInitState=TURN_ANGLE);
void turnDist(double rot, double distOfTurn, double speedOfTurn, enum currType postState, enum currType afterInitState=TURN_ANGLE);
void turnByTime(double rot, long timeOfTurn, double speedOfTurn, enum currType postState, enum currType afterInitState=TURN_TIME);
void moveDist(double dir, double distOfMove, double speedOfTurn, enum currType postState, enum currType afterInitState=MOVE_DIST);

//* ------------------------------------------- OBJECT INITIALISATIONS -------------------------------------------

//* DRIVEBASE SETUP
Motor MotorL(12, 13, 0, 1); 
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
const int L0XNum = 5;
VL53L0X lidarsl0x[L0XNum];
int l0x_readings[L0XNum] = {defaultLidarReading, defaultLidarReading, defaultLidarReading, defaultLidarReading, defaultLidarReading};
const int l0x_pins[L0XNum] = {3, 5, 6, 1, 2};
String l0x_labels[L0XNum] = {"FRONT: ", "FRONT LEFT: ", "LEFT: ", "RIGHT: ", "FRONT RIGHT: "}; //for print debugging
namespace L0X {
    enum L0X {FRONT, FRONT_LEFT, LEFT, RIGHT, FRONT_RIGHT };
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

//* ------------------------------------------- USER-DEFINED VARIABLES -------------------------------------------

//^ DEBUG
long beforeEntireLoopTimeMicros, afterEntireLoopTimeMicros;
long beforeLidarLoopTimeMicros, afterLidarLoopTimeMicros;
long beforeTCSLoopTimeMicros, afterTCSLoopTimeMicros;
long beforeEachTCSLoopTimeMicros[tcsNum], afterEachTCSLoopTimeMicros[tcsNum];
double debugLDist, debugRDist;
long testTimerMillis;
bool ledOn = false;
int greenState;

//^ LOGIC TIMINGS
long lastSerialPiSendMillis = millis();

//^ ESSENTIALS
currType curr = TCS_LINETRACK;
double steer = 0, rotation = 0;
double rpm = 100, lt_rpm = 100;
int serialState = 0, task = 0;
namespace Pi {
    enum Pi
    {
        LINETRACK = 0,
        SWITCH_OFF = 9
    };
}

//^ LINETRACK VARIABLES
int caze = 0;
double left = 1, right = 1;

//^ FORCED VARIABLES
bool forcedTurnAlrInit = false;
currType postForcedDistCase = EMPTY_LINETRACK;
double startForcedDistL, startForcedDistR, currForcedDist;
double wantedForcedDist = 0;
double forcedDirection = 0;
double forcedSpeed = 0;
long forcedTurnTime = 0, startTurnTime;

//^ ALIGNING SWEEP LINE GAP
int alignSweepState = 0;
bool lineAligned = true;
// bool endLineGap = true;
long afterGapMillis;
double prevTurnedAlignSweepDistL, prevTurnedAlignSweepDistR;
double alignSweepRotation;

int linegapSweepState = 0;
double prevTurnedLinegapSweepDistL, prevTurnedLinegapSweepDistR;
bool endLineGap = true;
double linegapSweepRotation;

//* ------------------------------------------ START SETUP -------------------------------------------

void setup() 
{
    Serial.println("----- void setup begin -----");

    //^ PIO
    pinMode(SWTPIN, INPUT);
    pinMode(PICOLEDPIN, OUTPUT);
    pinMode(LEDPIN, OUTPUT);
    digitalWrite(LEDPIN, HIGH);
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
    Serial1.begin(115200);
    while (!Serial1) delay(10); 
    Serial.println("Pi serial initialised");

    //^ MULTIPLEXERS
    Wire.setSDA(SDAPIN);
    Wire.setSCL(SCLPIN);
    Wire.begin();
    Wire.setClock(400000);
    Serial.println("Top multiplexer initialised");
    // Wire1.setSDA(SDA1PIN);
    // Wire1.setSCL(SCL1PIN);
    // Wire1.begin();
    // Wire1.setClock(400000);
    // Serial.println("Bottom multiplexer initialised");

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

    // //^ TCS
    // // follows the numbering 
    // // | 0 1 2 3 4 |
    // // |    5 6    |
    // for (int i = 1; i < 3; i++) {
    //     tcaselect2(tcs_pins[i]);
    //     while (!tcs[i].begin(TCSADR, &Wire1)) { Serial.println("ERROR: TCS34725 No. "); Serial.print(i); Serial.println(" NOT FOUND!"); }
    // }
    // Serial.println("TCS sensors initialised");

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
    digitalWrite(LEDPIN, HIGH);
    
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

    // //* TCS READINGS
    // #if debugLoopTime
    // beforeTCSLoopTimeMicros = micros();
    // #endif
    // for (int i = 1; i < 3; i++) { //^ only using 2 to linetrack right now
    //     tcaselect2(tcs_pins[i]);
    //     #if debugLoopTime
    //     beforeEachTCSLoopTimeMicros[i] = micros();
    //     #endif
    //     tcs[i].getRawData(&tcsSensors[i].r, &tcsSensors[i].g, &tcsSensors[i].b, &tcsSensors[i].c);
    //     rgb_to_hsv(i);
    //     #if debugLoopTime
    //     afterEachTCSLoopTimeMicros[i] = micros();
    //     #endif
    // }
    // #if debugLoopTime
    // afterTCSLoopTimeMicros = micros();
    // #endif

    if (!digitalRead(SWTPIN)) 
    {
        // tcsAnalyse();
        serialEvent();
        // ledOn = true;

        //* ------------------------------------------- PI TASK HANDLED -------------------------------------------
        
        if (curr != STOP) {
            switch (task)
            {
                case 0: //EMPTY LINETRACK
                    // if (((curr == MOVE_DIST && postForcedDistCase == EMPTY_LINETRACK) || curr == AFTER_ALIGN_SWEEP) && endLineGap) { curr = EMPTY_LINETRACK; }
                    if (curr == LINEGAP) { break; }
                    if (curr == ALIGN_SWEEP || curr == AFTER_ALIGN_SWEEP) { break; }
                    if (curr == MOVE_DIST || curr == TURN_ANGLE || curr == TURN_TIME) { break; }
                    if (curr == LEFT_GREEN || curr == RIGHT_GREEN || curr == DOUBLE_GREEN) { break; }
                    curr = EMPTY_LINETRACK;
                    break;

                case 1:
                    // curr = STOP;
                    if (curr == EMPTY_LINETRACK || curr == LINEGAP){
                        moveDist(1, 3*3, 100, LEFT_GREEN);
                    }
                    break;

                case 2:
                    // curr = STOP;
                    if (curr == EMPTY_LINETRACK || curr == LINEGAP){
                        moveDist(1, 3*3, 100, RIGHT_GREEN);
                    }
                    break;

                case 3:
                    if (curr == EMPTY_LINETRACK || curr == LINEGAP){
                        moveDist(1, 3*3, 100, DOUBLE_GREEN);
                    }
                    break;

                case 4:
                    curr = RED;
                    break;

                case 10:
                    if (curr == ALIGN_SWEEP || curr == AFTER_ALIGN_SWEEP) { break; }
                    if (curr == MOVE_DIST || curr == TURN_ANGLE) { break; }
                    if (millis() - afterGapMillis > 200) {
                        curr = ALIGN_SWEEP;
                        alignSweepState = 0;
                    }
                    break;

                case 12:
                    if (curr == EMPTY_LINETRACK) {
                        curr = LINEGAP;
                        linegapSweepState = 0;
                        linegapSweepRotation = 0;
                    }
                    break;
            }
        }

        //* ------------------------------------------- CURRENT ACTION HANDLED -------------------------------------------
        switch (curr)
        {
            case TCS_LINETRACK: //^ unused
                // left = grayPercent(1);
                // right = grayPercent(2);
                left = isBlack(1);
                right = isBlack(2);
                Serial.println("running tcs lt");
                // steer = (left-right)>0 ? pow(left - right, 0.5) : -pow(left - right, 0.5);
                if (left && right) {MotorL.setVal(80); MotorR.setVal(80); caze = 1;}
                else if (right){ MotorL.setVal(80); MotorR.setVal(-80); caze = 2;}
                else if (left) { MotorL.setVal(-80); MotorR.setVal(80); caze = 3;}
                else {MotorL.setVal(80); MotorR.setVal(80); caze = 4;}
                // Robawt.setSteer(lt_rpm, steer);
                Serial.print("Case: "); Serial.println(caze);
                break;

            case EMPTY_LINETRACK:
                send_pi(Pi::LINETRACK);
                Robawt.setSteer(rpm, rotation);
                Serial.println("running empty linetrack");
                Serial.print("rpm: "); Serial.println(rpm);
                Serial.print("rotation: "); Serial.println(rotation);
                break;

            case LEFT_GREEN:
                // turnAngle(-0.8, 225, 100, STOP);
                turnByTime(-0.8, 1000, 100, EMPTY_LINETRACK);
                break;

            case RIGHT_GREEN:
                // switch (greenState) {
                //     case 0:
                //         startTurnTime = millis();
                //         greenState ++;
                //         break;

                //     case 1:
                //         if (millis() - startTurnTime > 1000) {
                //             curr = STOP;
                //         }
                //         else { Robawt.setSteer(100, 0.80); }
                //         break;
                // }

                // turnAngle(0.8, 225, 100, STOP);
                turnByTime(0.8, 1000, 100, EMPTY_LINETRACK);
                break;

            case DOUBLE_GREEN:
                turnAngle(1, 207*3, 200, POST_DOUBLE_GREEN);
                break;

            case POST_DOUBLE_GREEN:
                Robawt.resetPID();
                curr = EMPTY_LINETRACK;
                break;

            case RED:
                // moveDist(1, 3*3, 100, STOP);
                curr = STOP;
                break;

            case ALIGN_SWEEP:
                #if debugAlignSweep
                Serial.print("aligned line: "); Serial.print(lineAligned);
                #endif
                #if debugWithLED
                ledOn = true;
                #endif
                switch (alignSweepState)
                {
                    case 0: //^ left turn
                        lineAligned = false;
                        turnAngle(-1, 80, ALIGN_SWEEP, ALIGN_SWEEP);
                        alignSweepState++;
                        // [[fallthrough]];
                        break;
                    case 1:
                        Robawt.setSteer(30, forcedDirection);
                        currForcedDist = getRotated(startForcedDistL, startForcedDistR);
                        #if debugAlignSweep
                        Serial.print(" curr dist: "); Serial.print(currForcedDist);
                        Serial.print(" wanted dist: "); Serial.println(wantedForcedDist);
                        #endif
                        if (currForcedDist >= wantedForcedDist) {
                            alignSweepRotation = 0;
                            prevTurnedAlignSweepDistL = currForcedDist;
                            alignSweepState++;
                            // [[fallthrough]];
                        } else if (lineAligned) {
                            curr = STOP;
                            Robawt.stop();
                            alignSweepRotation = -1;
                            prevTurnedAlignSweepDistL = currForcedDist;
                            alignSweepState++; 
                            // [[fallthrough]];
                        // } else {
                        }
                        break;
                        // }

                    case 2: //^ turn back right to the original position
                        turnDist(1, prevTurnedAlignSweepDistL, ALIGN_SWEEP, ALIGN_SWEEP);
                        alignSweepState++;
                        // [[fallthrough]];
                        break;
                    case 3:
                        lineAligned = false;
                        Robawt.setSteer(30, forcedDirection);
                        currForcedDist = getRotated(startForcedDistL, startForcedDistR);
                        #if debugAlignSweep
                        Serial.print(" curr dist: "); Serial.print(currForcedDist);
                        Serial.print(" wanted dist: "); Serial.println(wantedForcedDist);
                        #endif
                        if (currForcedDist >= wantedForcedDist) { 
                            // Robawt.stop();
                            alignSweepState++; 
                            // testTimerMillis = millis();
                            // [[fallthrough]];
                        }
                        // } else {
                        break;
                        // }

                    case 4: //^ right turn
                        // Robawt.setSteer(0, 0);
                        lineAligned = false;
                        turnDist(1, prevTurnedAlignSweepDistL, ALIGN_SWEEP, ALIGN_SWEEP);
                        // if (millis() - testTimerMillis > 500) { 
                        alignSweepState++; 
                        // }
                        // [[fallthrough]];
                        break;
                    case 5:
                        Robawt.setSteer(30, forcedDirection);
                        currForcedDist = getRotated(startForcedDistL, startForcedDistR);
                        #if debugAlignSweep
                        Serial.print(" curr dist: "); Serial.print(currForcedDist);
                        Serial.print(" wanted dist: "); Serial.println(wantedForcedDist);
                        #endif
                        if (currForcedDist >= wantedForcedDist) {
                            prevTurnedAlignSweepDistR = currForcedDist;
                            alignSweepState++;
                            // [[fallthrough]];
                        } else if (lineAligned) { 
                            curr = STOP;
                            Robawt.stop();
                            alignSweepRotation = 1;
                            prevTurnedAlignSweepDistR = currForcedDist;
                            alignSweepState = 8; 
                            // testTimerMillis = millis();
                            // break;
                        }
                        // } else {
                        break;
                        // }

                    case 6: //^ turn back left to original position
                        turnDist(-1, prevTurnedAlignSweepDistR, ALIGN_SWEEP, ALIGN_SWEEP);
                        alignSweepState++;
                        // [[fallthrough]];
                        break;
                    case 7:
                        lineAligned = false;
                        Robawt.setSteer(30, forcedDirection);
                        currForcedDist = getRotated(startForcedDistL, startForcedDistR);
                        #if debugAlignSweep
                        Serial.print(" curr dist: "); Serial.print(currForcedDist);
                        Serial.print(" wanted dist: "); Serial.println(wantedForcedDist);
                        #endif
                        if (currForcedDist >= wantedForcedDist) { 
                            // Robawt.stop();
                            alignSweepState++;
                            // testTimerMillis = millis();
                            // [[fallthrough]];
                        }
                        // } else {
                        break;
                        // }

                    case 8: //^ turn to required position
                        lineAligned = false;
                        // Robawt.setSteer(0, 0);
                        // if (millis() - testTimerMillis > 500) {
                            switch ((int)alignSweepRotation)
                            {
                                case 0:
                                    Robawt.setSteer(rpm, 0);
                                    curr = STOP;
                                    break;
                                case -1:
                                    turnDist(-1, prevTurnedAlignSweepDistL/2, 100, STOP);
                                    alignSweepRotation = 0;
                                    break;
                                case 1:
                                    turnDist(-1, prevTurnedAlignSweepDistR/2, 100, STOP);
                                    alignSweepRotation = 0;
                                    break;
                            }
                        // }
                        break;
                }
                break;

            case AFTER_ALIGN_SWEEP:
                // curr = STOP;
                #if debugWithLED
                ledOn = true;
                #endif
                send_pi(Pi::LINETRACK);
                moveDist(1, 20, 100, EMPTY_LINETRACK);
                afterGapMillis = millis();
                break;

            case LINEGAP:
                #if debugLinegapSweep
                Serial.print("end line gap: "); Serial.print(endLineGap);
                #endif
                // #if debugWithLED
                // ledOn = true;
                // #endif
                switch (linegapSweepState)
                {
                    case 0: //^ left turn
                        endLineGap = false;
                        turnAngle(-1, 240, 100, LINEGAP, LINEGAP);
                        linegapSweepState++;
                        // [[fallthrough]];
                        break;
                    case 1:
                        Robawt.setSteer(forcedSpeed, forcedDirection);
                        currForcedDist = getRotated(startForcedDistL, startForcedDistR);
                        #if debugLinegapSweep
                        Serial.print(" curr dist: "); Serial.print(currForcedDist);
                        Serial.print(" wanted dist: "); Serial.println(wantedForcedDist);
                        #endif
                        if (currForcedDist >= wantedForcedDist) {
                            linegapSweepRotation = 0;
                            prevTurnedLinegapSweepDistL = currForcedDist;
                            linegapSweepState++;
                            // [[fallthrough]];
                        } else if (endLineGap) {
                            curr = STOP;
                            Robawt.stop();
                            linegapSweepRotation = -1;
                            prevTurnedLinegapSweepDistL = currForcedDist;
                            linegapSweepState++; 
                        }
                        break;

                    case 2: //^ turn back right to the original position
                        turnDist(1, prevTurnedLinegapSweepDistL, 100, LINEGAP, LINEGAP);
                        linegapSweepState++;
                        // [[fallthrough]];
                        break;
                    case 3:
                        endLineGap = false;
                        Robawt.setSteer(forcedSpeed, forcedDirection);
                        currForcedDist = getRotated(startForcedDistL, startForcedDistR);
                        #if debugLinegapSweep
                        Serial.print(" curr dist: "); Serial.print(currForcedDist);
                        Serial.print(" wanted dist: "); Serial.println(wantedForcedDist);
                        #endif
                        if (currForcedDist >= wantedForcedDist) { 
                            // Robawt.stop();
                            linegapSweepState++; 
                            // testTimerMillis = millis();
                            // [[fallthrough]];
                        }
                        // } else {
                        break;
                        // }

                    case 4: //^ right turn
                        endLineGap = false;
                        turnDist(1, prevTurnedLinegapSweepDistL, 100, LINEGAP, LINEGAP);
                        // if (millis() - testTimerMillis > 500) { 
                        linegapSweepState++; 
                        // }
                        // [[fallthrough]];
                        break;
                    case 5:
                        Robawt.setSteer(forcedSpeed, forcedDirection);
                        currForcedDist = getRotated(startForcedDistL, startForcedDistR);
                        #if debugLinegapSweep
                        Serial.print(" curr dist: "); Serial.print(currForcedDist);
                        Serial.print(" wanted dist: "); Serial.println(wantedForcedDist);
                        #endif
                        if (currForcedDist >= wantedForcedDist) {
                            prevTurnedLinegapSweepDistR = currForcedDist;
                            linegapSweepState++;
                            // [[fallthrough]];
                        } else if (endLineGap) { 
                            curr = STOP;
                            Robawt.stop();
                            linegapSweepRotation = 1;
                            prevTurnedLinegapSweepDistR = currForcedDist;
                            linegapSweepState = 8; 
                            // testTimerMillis = millis();
                            // break;
                        }
                        // } else {
                        break;
                        // }

                    case 6: //^ turn back left to original position
                        turnDist(-1, prevTurnedLinegapSweepDistR, 100, LINEGAP, LINEGAP);
                        linegapSweepState++;
                        // [[fallthrough]];
                        break;
                    case 7:
                        endLineGap = false;
                        Robawt.setSteer(forcedSpeed, forcedDirection);
                        currForcedDist = getRotated(startForcedDistL, startForcedDistR);
                        #if debugLingapSweep
                        Serial.print(" curr dist: "); Serial.print(currForcedDist);
                        Serial.print(" wanted dist: "); Serial.println(wantedForcedDist);
                        #endif
                        if (currForcedDist >= wantedForcedDist) { 
                            linegapSweepState++;
                        }
                        // } else {
                        break;
                        // }

                    case 8: //^ turn to required position
                        endLineGap = false;
                        switch ((int)linegapSweepRotation)
                        {
                            case 0:
                                Robawt.setSteer(rpm, 0);
                                curr = STOP;
                                break;
                            case -1:
                                turnDist(-1, prevTurnedLinegapSweepDistL/2, 100, STOP);
                                linegapSweepRotation = 0;
                                break;
                            case 1:
                                turnDist(-1, prevTurnedLinegapSweepDistR/2, 100, STOP);
                                linegapSweepRotation = 0;
                                break;
                        }
                        // }
                        break;
                }
                break;

            //* ------------------------------------------- FORCED MOVEMENTS -------------------------------------------
                
            case STOP:
                Robawt.stop();
                break;

            case MOVE_DIST:
                #if debugWithLED
                ledOn = true;
                #endif
                currForcedDist = getRotated(startForcedDistL, startForcedDistR);
                if (currForcedDist >= wantedForcedDist) { curr = postForcedDistCase; }
                else { Robawt.setSteer(forcedDirection * forcedSpeed, 0); }
                break;

            case TURN_ANGLE:
                #if debugWithLED
                ledOn = true;
                #endif
                currForcedDist = getRotated(startForcedDistL, startForcedDistR);
                if (currForcedDist >= wantedForcedDist) { curr = postForcedDistCase; }
                else { Robawt.setSteer(forcedSpeed, forcedDirection); }
                break;

            case TURN_TIME:
                #if debugWithLED
                ledOn = true;
                #endif
                if (millis() - startTurnTime > forcedTurnTime) { curr = postForcedDistCase; }
                else { Robawt.setSteer(forcedSpeed, forcedDirection); } 
                break;

            default:
                #if debugWithLED
                ledOn = true;
                #endif
                Serial.println("ERROR: variable curr undefined, default case triggered");
                break;
        }

        #if debugState
        Serial.print("----- curr: "); Serial.print(curr); Serial.print(" task: "); Serial.print(task); Serial.println(" -----");
        Serial.print("alignSweepState: "); Serial.println(alignSweepState);
        #endif

    } else {
        //* ------------------------------------------- SWITCH OFF -------------------------------------------

        Robawt.setSteer(0, 0);
        Robawt.resetPID();
        curr = EMPTY_LINETRACK;
        alignSweepState = 0;
        ledOn = false;
        send_pi(Pi::SWITCH_OFF);
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

    #if debugLidarReadings
    for (int i = 0; i < L0XNum; i++) {
        Serial.print(l0x_labels[i]); 
        Serial.print(l0x_readings[i]); Serial.print(" | ");
    }
    for (int i = 0; i < L1XNum; i++) {
        Serial.print(l1x_labels[i]);
        Serial.print(l1x_readings[i]); Serial.print("\t");
    }
    Serial.println();
    #endif
}

//* ------------------------------------------- USER DEFINED FUNCTIONS -------------------------------------------

//* COMMUNICATIONS

void serialEvent() //Pi to pico serial
{   
    while (Serial1.available()) 
    {
        int serialData = Serial1.read();
        if (serialData == 255 || serialData == 254 || serialData == 253 || serialData == 252 || serialData == 251) {
            serialState = (int)serialData;
            #if debugSerial
            Serial.print("Serial State: "); Serial.println(serialState);
            #endif
        } else {
            switch (serialState) 
            {
                case 255:
                    rotation = ((double)(serialData)-90)/90;
                    break;
                case 254:
                    rpm = (double)serialData;
                    break;
                case 253:
                    task = (int)serialData;
                    break;
                case 252:
                    lineAligned = (bool)serialData;
                    break;
                case 251:
                    endLineGap = (bool)serialData;
                    break;
            }
            #if debugSerial
            Serial.print("----- Data: "); Serial.print(serialData); Serial.println(" -----");
            Serial.print("Rotation: "); Serial.print(rotation); Serial.print(" RPM: "); Serial.print(rpm); Serial.print(" Task: "); Serial.println(task); 
            Serial.print("lineAligned: "); Serial.print(lineAligned); Serial.print(" endLineGap: "); Serial.println(endLineGap);
            #endif
        }
    }
}

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

//* SERVO FUNCTIONS

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

//* TCS FUNCTIONS

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
    if (grayPercent(i) < 0.5) { return true; }
    else { return false; }
}

bool isGreen(int i) 
{
    return (tcs_lgreen[i] < tcsSensors[i].hue) && (tcs_ugreen[i] > tcsSensors[i].hue);
}

bool isRed(int i) 
{
    return ((tcs_lred1[i] < tcsSensors[i].hue) && (tcs_ured1[i] > tcsSensors[i].hue)) || ((tcs_lred2[i] < tcsSensors[i].hue) && (tcs_ured2[i] > tcsSensors[i].hue));
}

bool isSilver(int i)
{
    return false;
}

void tcsAnalyse(int i)
{
    tcsSensors[i].green = isGreen(i);
    tcsSensors[i].red = isRed(i);
    tcsSensors[i].black = isBlack(i);
    tcsSensors[i].silver = isSilver(i);
}

//* DRIVEBASE FUNCTIONS

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

void turnAngle(double rot, double angleOfTurn, double speedOfTurn, enum currType postState, enum currType afterInitState) //in degrees
{
    //only support: 0.5 OR 1 rot, speed 30 rpm, < 180 degrees. note: angle is always positive
    forcedDirection = rot;
    forcedSpeed = speedOfTurn;
    startForcedDistL = MotorL.getDist();
    startForcedDistR = MotorR.getDist();
    curr = afterInitState;
    if (abs(rot) == 1) {
        wantedForcedDist = angleOfTurn / 5.95;
        wantedForcedDist += constraint((angleOfTurn - 90)/60, -0.5, 0.5); //to account for discrepancies in smaller angle turns: decel and accel requires a tighter distance to work with, as well as larger distance means higher uncertainty between the unsynced steering
    } else { //for one wheel turns
        wantedForcedDist = angleOfTurn / 5.75;
        wantedForcedDist += constraint(((angleOfTurn - 90)/90), -0.5, 0.5);
    }
    postForcedDistCase = postState;
}

void turnByTime(double rot, long timeOfTurn, double speedOfTurn, enum currType postState, enum currType afterInitState){
    forcedDirection = rot;
    forcedTurnTime = timeOfTurn;
    startTurnTime = millis();
    forcedSpeed = speedOfTurn;
    curr = afterInitState;
    postForcedDistCase = postState;
}

void turnDist(double rot, double distOfTurn, double speedOfTurn, enum currType postState, enum currType afterInitState)
{
    forcedDirection = rot;
    forcedSpeed = speedOfTurn;
    startForcedDistL = MotorL.getDist();
    startForcedDistR = MotorR.getDist();
    curr = afterInitState;
    wantedForcedDist = distOfTurn;
    postForcedDistCase = postState;
}

void moveDist(double dir, double distOfMove, double speedOfTurn, enum currType postState, enum currType afterInitState) //in cm
{
    //only support: speed 30 rpm. note: put .0 if whole number, or problems will happen
    forcedSpeed = speedOfTurn;
    forcedDirection = dir; //1 or -1 for forward and backward respectively
    startForcedDistL = MotorL.getDist();
    startForcedDistR = MotorR.getDist();
    curr = afterInitState;
    wantedForcedDist = distOfMove * 1.47;
    postForcedDistCase = postState;
}

//* LIDAR FUNCTIONS

//~ Inside evac

// bool ball_present() {
//     //+45 for the diff sensors' offset physically
//     return ((l0x_readings[L0X::FRONT]+45 - l1x_readings[L1X::FRONT_BOTTOM]) > 30 && l1x_readings[L1X::FRONT_BOTTOM] < 80);
// }

// bool wall_present() {
//     return (l0x_readings[L0X::FRONT]+45 < 90 && l1x_readings[L1X::FRONT_BOTTOM] < 90);
// }

// bool OOR_present() {
//     return ((front_see_infinity() && right_see_wall()) //if front sees out, and right either sees wall or OOR
//     || (front_see_infinity() && (left_see_infinity() || (frontLeft_see_infinity() && left_see_out())))); //if left and front sees out
// }

// bool wallgap_present() {
//     return (l0x_readings[L0X::LEFT] > 1680 || l0x_readings[L0X::FRONT_LEFT] > 1680);
// }

// bool front_see_infinity() {
//     return (l0x_readings[L0X::FRONT] > 1680);
// }

// bool left_see_infinity() {
//     return  (l0x_readings[L0X::LEFT] > 1680);
// }

// bool right_see_infinity() {
//     return  (l0x_readings[L0X::RIGHT] > 1680);
// }

// bool frontLeft_see_infinity() {
//     return (l0x_readings[L0X::FRONT_LEFT] > 1680);
// }

//~ Exiting evac

// bool left_see_out() {
//     return (l0x_readings[L0X::LEFT] > 400);
// }

// bool front_see_out() {
//     return (l0x_readings[L0X::FRONT] > 400);
// }

// bool right_see_out() {
//     return (l0x_readings[L0X::FRONT] > 600);
// }

// bool frontLeft_see_out() {
//     return (l0x_readings[L0X::FRONT_LEFT] > 600);
// }

// bool right_see_wall() {
//     return (l0x_readings[L0X::RIGHT] < 500);
// }

// bool left_see_closewall() {
//     return (l0x_readings[L0X::LEFT] < 200);
// }

// bool right_see_closewall() {
//     return (l0x_readings[L0X::RIGHT] < 200);
// }

// bool evacExitFrontValSeeInfinity() {
//     return (evacExitFrontVal > 1680);
// }

// bool left_see_reallyclosewall() {
//     return (l0x_readings[L0X::LEFT] < 150);
// }

// bool right_see_reallyclosewall() {
//     return (l0x_readings[L0X::RIGHT] < 150);
// }