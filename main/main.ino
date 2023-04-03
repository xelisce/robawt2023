//* ------------------------------------------- PREPROCESSER DIRECTIVES -------------------------------------------
#include <Arduino.h>
#include <Wire.h>
#include "VL53L1X.h"
#include "VL53L0X.h"
#include "Vroom.h"
#include <Servo.h>

//* DEBUG SETTINGS
//^ Runs with code
#define debug_serial 0
#define debug_led 1
#define debug_looptime 0
#define debug_lidars 1
#define debug_curr 1
#define debug_distance 1
//^ Runs without normal code
#define loop_movetime 0
#define loop_movedistance 0
#define loop_pickball 0
#define loop_pickcube 0
#define loop_depositalive 0
#define loop_depositdead 0

//* ADDRESSES
#define TCAADDR 0x70

//* CONSTANT PINS
#define TNYPIN1 9
#define TNYPIN2 8
#define TX0PIN 16
#define RX0PIN 17
#define SDAPIN 4
#define SCLPIN 5
#define SWTPIN 28
#define LEDPIN 3
#define ONBOARDLEDPIN 25

//* ------------------------------------------- START CODE -------------------------------------------

//* OBJECT INITIALISATIONS
Motor MotorR(13, 12, 19, 18); //M2 swapped
Motor MotorL(10, 11, 1, 0); //M1
Vroom Robawt(&MotorL, &MotorR);
VL53L0X lidarsl0x[4];
VL53L1X lidarsl1x[1];
Servo servos[6];

//* MOTOR ENCODERS
void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }

//* LIDARS SETUP
//^ VL53L0X
int l0x_readings[4] = {0, 0, 0, 0};
const int l0x_pins[4] = {4, 3, 1, 5};
String l0x_labels[4] = {"FRONT: ", "FRONT LEFT: ", "LEFT: ", "RIGHT: "};
namespace L0X {
    enum L0X { FRONT, FRONT_LEFT, LEFT, RIGHT };
}
//^ VL53L1X
int l1x_readings[1] = {0};
const int l1x_pins[1] = {2};
String l1x_labels[1] = {"FRONT BOTTOM: "};
namespace L1X {
    enum L1X { FRONT_BOTTOM };
}
//^ Debugging Lidars
const int l0x_start = L0X::FRONT, //first lidar
    l0x_stop = L0X::RIGHT; //last l0x lidar
const int l1x_start = L1X::FRONT_BOTTOM, //first l1x lidar
    l1x_stop = L1X::FRONT_BOTTOM; //last l1x lidar

//* SERVOS SETUP
double servos_angle[6] = {180, 0, 135, 90, 140, 0}; //basic states initialised
const double servos_max_angle[6] = {180, 300, 300, 300, 180, 180};
const int servos_pin[6] = {27, 26, 22, 2, 21, 20};
bool servos_change = false;
namespace Servos {
  enum Servos { ARM, LEFT, RIGHT, SORT, ALIVE, DEAD };
}
//^ Debugging servos
const int servos_start = Servos::ARM, //first servo
    servos_stop = Servos::ALIVE; //last servo

//* VARIABLES

//^ Debug
long firstLoopTimeMicros,
    beforeLidarLoopTimeMicros,
    afterLidarLoopTimeMicros;
bool led_on = false;

//^ Debug loops
long startDistanceMillis;
long int startDistanceValL, 
    startDistanceValR;

//^ 135
bool left135 = false,
    right135 = false;
long last135Millis, 
    start135Millis;

//^ 90
long start90Millis,
    lostGSMillis;

//^ Green squares
long startGSMillis;
double startGSDistL, 
    startGSDistR,
    distDoubleGSTravelled;

//^ Red
long startRedMillis;

//^ Movement and logic
double rotation = 0,
    rpm = 40;
int serialState = 0,
    task = 0, 
    prev_task = 0,
    curr = 0;

//^ Claw stuffz
int afterPickupState,
    pickType;

//^ Obstacle
int turn_dir,
    obstState;
int* sideObstDist;
double obstDist,
    obstDistL,
    obstDistR,
    obstDistTravelled,
    obstStartTurnBackDist,
    obstCurrDist;
float o_rotation = 0.5; //obstacle fixed rotation
long obst_time_start;
bool see_line = false;

//^ Blue cube
double prev_kit_rotation,
    kitStartDist,
    kitBeforeStraightDist,
    kitDistToReverse = 0,
    kitStartReverseDist,
    kitStartTurnBackDist,
    kitTurnBackDist,
    kit_distToTurn;

//^ Evac
bool in_evac = false;
long pickupStateTimer;
int pickupState = 0,
    depositState = 0;

//* ------------------------------------------- START SETUP -------------------------------------------

void setup() 
{
    Serial.begin(9600);
    // while (!Serial) delay(10); 
    Serial.println("USB serial initialised");

    pinMode(SWTPIN, INPUT_PULLDOWN);
    pinMode(ONBOARDLEDPIN, OUTPUT);
    pinMode(LEDPIN, OUTPUT);
    pinMode(TNYPIN1, INPUT);
    pinMode(TNYPIN2, INPUT);

    //^ PI SERIAL COMMS
    Serial1.setRX(RX0PIN);
    Serial1.setTX(TX0PIN);
    Serial1.begin(9600);
    while (!Serial1) delay(10); 
    Serial.println("Pi serial initialised");

    //^ MULTIPLEXER
    Wire.setSDA(SDAPIN);
    Wire.setSCL(SCLPIN);
    Wire.begin();
    Wire.setClock(400000); 

    //^ LIDAR INITIALISATIONS
    for (int i = l0x_start; i != (l0x_stop+1); i++) 
    {
        tcaselect(l0x_pins[i]);
        lidarsl0x[i].setTimeout(500);
        while (!lidarsl0x[i].init()) {
            Serial.print(l0x_labels[i]);
            Serial.print("at pin ");
            Serial.print(l0x_pins[i]);
            Serial.print(" - ");
            Serial.println("L0X failed to initialise");
        }
        lidarsl0x[i].startContinuous();
    }
    for (int i = l1x_start; i != (l1x_stop+1); i++) 
    {
        tcaselect(l1x_pins[i]);
        lidarsl1x[i].setTimeout(500);
        while (!lidarsl1x[i].init()) {
            Serial.print(l1x_labels[i]);
            Serial.print("at pin ");
            Serial.print(l1x_pins[i]);
            Serial.print(" - ");
            Serial.println("L1X failed to initialise");
        }
        lidarsl1x[i].setDistanceMode(VL53L1X::Medium);
        lidarsl1x[i].startContinuous(20);
    }

    //^ SERVO INITIALISATIONS
    for (int i = servos_start; i != (servos_stop+1); i++) {
        servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
        servos[i].attach(servos_pin[i], 500, 2500); // (pin, min, max)
    }

    //^ MOTOR ENCODERS
    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

//* ------------------------------------------- START LOOP -------------------------------------------

#if !loop_movetime && !loop_movedistance && !loop_pickball && !loop_pickcube && !loop_depositalive && !loop_depositdead
void loop() 
{

    //* COMMUNICATION UPDATES
    //  teensyEvent();
    serialEvent();

    //* SERVO UPDATES
    if (servos_change) {
        for (int i = servos_start; i != (servos_stop+1); i++) {
        servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
        }
        servos_change = false;
    }

    //* LIDAR READINGS
    #if debug_looptime
    beforeLidarLoopTimeMicros = micros();
    #endif
    for (int i = l0x_start; i != (l0x_stop+1); i++) 
    {
        tcaselect(l0x_pins[i]);
        if(lidarsl0x[i].available()) {
            l0x_readings[i] = lidarsl0x[i].readRangeMillimeters();
        }
    }
    for (int i = l1x_start; i != (l1x_stop+1); i++) 
    {
        tcaselect(l1x_pins[i]);
        if(lidarsl1x[i].dataReady()) {
            l1x_readings[i] = lidarsl1x[i].read(false);
        }
    }
    #if debug_looptime
    afterLidarLoopTimeMicros = micros();
    #endif

    //* SWITCH IS ON
    if (!digitalRead(SWTPIN))
    {

        //* TASK FROM PI
        switch (task) 
        {
            case 0: //^ empty linetrack
                if (in_evac) { break; }
                if (curr == 1) { 
                    //~ enter post left green mode after minimum turn time
                    if (millis() - startGSMillis > 200) { 
                        curr = 21; 
                        lostGSMillis = millis(); }
                } else if (curr == 2) {
                    //~ enter post right green mode after minimum turn time
                    if (millis() - startGSMillis > 200) { 
                        curr = 22; 
                        lostGSMillis = millis(); }
                } else if (curr == 3) {
                    //~ turn a full 180 deg for double green (dist tuned for 50 rpm, 1 rot)
                    distDoubleGSTravelled = abs(MotorL.getDist()-startGSDistL) + abs(MotorR.getDist()-startGSDistR);
                    if (distDoubleGSTravelled > 65) { curr = 0; }
                } else if (curr == 4) {
                    //~ drive forward till over red line
                    if (millis() - startRedMillis > 1000) { curr = 100; }
                } else if (curr == 5 || curr == 6) { 
                    //~ only return back to linetrack after 90 if speific time has passed
                    if (millis() - start90Millis > 500) { curr = 0; }
                } else if (curr == 21 || curr == 22) {
                    //~ post green continue to rotate
                    if (millis() - lostGSMillis > 300) { curr = 0; }
                } else if ((curr < 30 || curr > 33) && curr != 100 && (curr < 23 || curr > 25) && curr != 7 && curr != 8 && curr != 50) {
                    //~ if not in obstacle or on red line or not post blue or not current blue or not picking up stuff
                    curr = 0; }
                break;

            case 1: //^ left green
                if (in_evac) { break; }
                if (curr == 3) { break; } //~ double green
                if (curr > 29 && curr < 34) { break; } //~obstacle cases
                if (curr == 0) { startGSMillis = millis(); }
                curr = 1;
                break;

            case 2: //^ right green
                if (in_evac) { break; }
                if (curr == 3) { break; } //~ double green
                if (curr > 29 && curr < 34) { break; } //~obstacle cases
                if (curr == 0) { startGSMillis = millis(); }
                curr = 2;
                break;
 
            case 3: //^ double green
                if (in_evac) { break; }
                if (curr > 29 && curr < 34) { break; } //~obstacle cases
                if (curr == 0) { 
                    startGSDistL = MotorL.getDist();
                    startGSDistR = MotorR.getDist(); }
                curr = 3;
                break;
            
            case 4: //^ red line --> go
                if (in_evac) { break; }
                if (curr = 0) { startRedMillis = millis(); }
                curr = 4;
                break;

            case 5: //^ left 90 (not in use currently)
                if (in_evac) { break; }
                if (curr == 6) { break; }
                if (curr == 0) { start90Millis = millis(); }
                curr = 5;
                break;  

            case 6: //^ right 90 (not in use currently)
                if (in_evac) { break; }
                if (curr == 5) { break; }
                if (curr == 0) { start90Millis = millis(); }
                curr = 6;
                break;  

            case 7: //^ turning to blue
                // if (in_evac) { break; }
                if (curr == 0) { 
                    prev_kit_rotation = rotation; 
                    kitStartDist = pickMotorDist(prev_kit_rotation)}
                curr = 7;
                break;

            case 8: //^ blue centred 
                // if (in_evac) { break; }
                curr = 8;
                break;
        }   

        //* CURRENT ACTION HANDLED
        switch (curr)
        {

            //* LINETRACK CASES

            case 0: //^ empty linetrack
                #if debug_led
                led_on = false;
                #endif
                Robawt.setSteer(rpm, rotation);
                //~ Trigger obstacle
                if (l0x_readings[L0X::FRONT] < 40) {
                    curr = 30;
                    turn_dir = l0x_readings[L0X::LEFT] > l0x_readings[L0X::RIGHT] ? -1 : 1;
                    obstDist = MotorL.getDist();
                }
                break;

            case 1: //^ left green
                Robawt.setSteer(rpm, -0.6);
                break;

            case 2: //^ right green
                Robawt.setSteer(rpm, 0.6);
                break;
            
            case 3: //^ double green
                Robawt.setSteer(50, 1);
                break;

            case 4: //^ red --> go
                #if debug_led
                led_on = true;
                #endif
                Robawt.setSteer(30, 0);
                break;

            case 5: //^ turn left 90 (not in use currently)
                Robawt.setSteer(rpm, -1);
                #if debug_led
                led_on = true;
                #endif
                break;

            case 6: //^ turn right 90 (not in use currently)
                Robawt.setSteer(rpm, 1);
                #if debug_led
                led_on = true;
                #endif
                break;

            case 7: //^ turning to blue cube
                Robawt.setSteer(rpm, rotation);
                claw_down();
                claw_halfclose();
                kitBeforeStraightDist = pickMotorDist(prev_kit_rotation); // chooses between L or R motor encoder vals based on previous rotation
                if (ball_present()) { 
                    afterPickupState = 23; //go to post cube if cube is ever picked up
                    pickupState = 0;
                    pickType = 0;
                    curr = 50; } //pick up le cube
                break;

            case 8: //^ blue centred
                Robawt.setSteer(rpm, 0);
                claw_down();
                claw_halfclose();
                if (ball_present()) { 
                    afterPickupState = 23;
                    pickupState = 0;
                    pickType = 0;
                    Robawt.setSteer(0, 0);
                    Robawt.resetPID();
                    curr = 50; } 
                break;

            //* POST CASES

            case 21: //^ post left green
                Robawt.setSteer(rpm, -0.6);
                break;

            case 22: //^ post right green
                Robawt.setSteer(rpm, 0.6);
                break;

            case 23: //^ post cube - initialisating vars
                #if debug_led
                led_on = true;
                #endif
                kitStartReverseDist = pickMotorDist(prev_kit_rotation);
                kitDistToReverse = pickMotorDist(prev_kit_rotation) - kitBeforeStraightDist;
                curr = 24;
                break;

            case 24: //^ post cube reversing
                #if debug_led
                led_on = true;
                #endif
                Robawt.setSteer(-rpm, 0);
                if (fabs(pickMotorDist(prev_kit_rotation) - kitStartReverseDist) > kitDistToReverse) { 
                    kit_distToTurn = kitBeforeStraightDist - kitStartDist;
                    kitStartTurnBackDist = pickMotorDist(prev_kit_rotation);
                    curr = 25; }
                break;

            case 25: //^ post cube turning
                #if debug_led
                led_on = true;
                #endif
                Robawt.setSteer(-rpm, prev_kit_rotation);
                kitTurnBackDist =  abs(pickMotorDist(prev_kit_rotation)-kitStartTurnBackDist);
                if (kitTurnBackDist > kit_distToTurn) { curr = 0; }
                break;

            //* OBSTACLE LOGIC (30 - 33)

            case 30: //^ reversing after detecting obstacle
                Robawt.setSteer(-40, 0);
                Serial.println(MotorL.getDist() - obstDist);
                if (MotorL.getDist() - obstDist < -12) {
                    sideObstDist = turn_dir == 1 ? &l0x_readings[L0X::LEFT] : &l0x_readings[L0X::RIGHT]; //^ getting address of readings to constantly update the val
                    curr = 31; }
                break;

            case 31: //^ turning 90 degrees
                switch (obstState)
                {
                    case 0:
                        Robawt.setSteer(rpm, turn_dir);
                        if (*sideObstDist > 300) { obstState ++; }
                        break;

                    case 1:
                        Robawt.setSteer(rpm, turn_dir);
                        if (*sideObstDist < 300) { obstState ++; }
                        break;

                    case 2:
                        Robawt.setSteer(rpm, 0);
                        obstState = 0;
                        curr = 32;
                        obst_time_start = millis();
                        break;
                }
                Serial.print("Turn dir: ");
                Serial.print(turn_dir);
                Serial.print(" || Lidar reading: ");
                Serial.println(*sideObstDist);
                break;

            case 32: //^ going around obstacle
                switch (obstState)
                {
                    case 0:
                        Robawt.setSteer (25, 0);
                        if (*sideObstDist < 200) { obstState++; }
                        break;
                        
                    case 1:
                        Robawt.setSteer (25, 0);
                        if (*sideObstDist > 200) { obstState++; }
                        break;

                    case 2:
                        Robawt.setSteer (25, -o_rotation*turn_dir);
                        if (*sideObstDist < 200) { obstState++; }
                        break;

                    case 3:
                        Robawt.setSteer (25, -o_rotation*turn_dir);
                        if (*sideObstDist > 200) { obstState = 0; }
                        break;
                }
                Serial.print("Obstacle state: ");
                Serial.println(obstState);
                //~ Minimum obstacle turn time
                if (see_line && (millis() - obst_time_start) > 6000){
                    curr = 33;
                    obst_time_start = millis();
                    obstState = 0; 
                    obstStartTurnBackDist = pickMotorDist(turn_dir); }
                break;

            case 33: //^ turning back to line after obstacle
                Robawt.setSteer(30, turn_dir*0.5);
                obstCurrDist = pickMotorDist(turn_dir);
                if (obstCurrDist - obstStartTurnBackDist > 20) { curr = 0; }
                break;

            //* EVAC CASES

            case 50: //^ pickup
                switch (pickupState)
                {
                    case 0:
                        pickupStateTimer = millis();
                        pickupState ++;
                        break;

                    case 1:
                        if (pickType == 0) { claw_close_cube(); }
                        else { claw_close(); }
                        if (millis() - pickupStateTimer > 1000) {
                            pickupStateTimer = millis();
                            pickupState ++; }
                        break;

                    case 2:
                        claw_up();
                        if (millis() - pickupStateTimer > 1000) {
                            pickupStateTimer = millis();
                            pickupState ++; 
                            Robawt.setSteer(0, 0);
                            Robawt.resetPID(); }
                        break;

                    case 3: 
                        claw_open();
                        if (millis() - pickupStateTimer > 1000) {
                            pickupStateTimer = millis();
                            pickupState ++; }
                        break;

                    case 4:
                        claw_down();
                        if (millis() - pickupStateTimer > 1000) {
                            pickupStateTimer = millis();
                            pickupState = 0; 
                            curr = afterPickupState; }
                        break;
                }
                Serial.print("Pickup state: ");
                Serial.println(pickupState);
                break;

            case 100: //^ red --> stop
                Robawt.setSteer(0, 0);
                Robawt.reset();
                #if debug_led
                led_on = false;
                #endif
                break;
        }

    //* SWITCH IS OFF
    } else {
        Robawt.setSteer(0, 0);
        Robawt.reset();
        curr = 0;

        claw_down();
        claw_open();
    }

    //* DEBUG PRINTS
    
    #if debug_lidars
    for (int i = l0x_start; i != (l0x_stop+1); i++) {
        Serial.print(l0x_labels[i]);
        Serial.print(l0x_readings[i]);
        // if (lidarsl0x[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
        Serial.print(" || ");
    }
    for (int i = l1x_start; i != (l1x_stop+1); i++) {
        Serial.print(l1x_labels[i]);
        Serial.print(l1x_readings[i]);
        // if (lidarsl1x[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
        Serial.print(" || ");
    }
    Serial.println();
    #endif

    #if debug_led
    if (led_on) { digitalWrite(ONBOARDLEDPIN, HIGH); }
    else { digitalWrite(ONBOARDLEDPIN, LOW); }
    led_on = false;
    #endif

    #if debug_looptime
    Serial.print("Time loop 1 (micros): ");
    Serial.print(micros() - firstLoopTimeMicros);
    firstLoopTimeMicros = micros();
    Serial.print(" || ");
    Serial.print("Lidar reading time (micros): ");
    Serial.print(afterLidarLoopTimeMicros-beforeLidarLoopTimeMicros);
    Serial.println();
    #endif

    #if debug_distance
    Serial.print("Motor L: ");
    Serial.print(MotorL.getDist());
    Serial.print(" || Motor R: ");
    Serial.print(MotorR.getDist());
    Serial.println();
    #endif

    #if debug_curr
    Serial.print("Curr: ");
    Serial.println(curr);
    #endif
}
#endif

//* ------------------------------------------- USER DEFINED FUNCTIONS -------------------------------------------

//* COMMUNICATIONS

void teensyEvent() //Teensy to pico binary
{
    int pin1State = digitalRead(TNYPIN1);
    int pin2State = digitalRead(TNYPIN2);
    Serial.print("pin1state: " );
    Serial.println(pin1State);
    Serial.print("pin2state: " );
    Serial.println(pin2State);
    if (pin1State && pin2State) { //double black 11
        curr = 0;
    } else if (!in_evac && !pin1State && pin2State) { //left 01
        curr = 1;
    } else if (!in_evac && pin1State && !pin2State) { //right 10
        curr = 2;
    } else {
        curr = 3;
    }
}

void serialEvent() //Pi to pico serial
    {
    while (Serial1.available()) 
    {
        int serialData = Serial1.read();
        if (serialData == 255 || serialData == 254 || serialData == 253 || serialData == 252) {
            serialState = (int)serialData;
            #if debug_serial
            Serial.print("Serial State: ");
            Serial.println(serialState);
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
                    see_line = (bool)serialData;
                    break;
            }
            #if debug_serial
            Serial.print("Data: ");
            Serial.println(serialData);
            #endif
        }
    }
}

void send_pi(int i) { //Pico to pi serial
  Serial1.println(i);
}

void tcaselect(uint8_t i)  //I2C Multiplexer: TCA9548A
{
  if (i > 0 && i < 7) {
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
  }
}

//* CLAW & SERVO FUNCTIONS

int pwmangle(double angle, double max_angle) //Servo PWM
{
  return (int)(angle/max_angle * 2000 + 500);
}

void claw_open() {
  servos_angle[Servos::RIGHT] = 135; 
  servos_angle[Servos::LEFT] = 0;
  servos_change = true;
}

void claw_close() { //ball
  servos_angle[Servos::RIGHT] = 30;
  servos_angle[Servos::LEFT] = 100;
  servos_change = true;
}

void claw_close_cube() {
  servos_angle[Servos::RIGHT] = 25;
  servos_angle[Servos::LEFT] = 105;
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
  servos_angle[Servos::ARM] = 180;
  servos_change = true;
}

void claw_halfclose() {
  servos_angle[Servos::RIGHT] = 80;
  servos_angle[Servos::LEFT] = 50;
  servos_change = true;
}

void alive_up() {
    servos_angle[Servos::ALIVE] = 40;
    servos_change = true;
}

void alive_down() {
    servos_angle[Servos::ALIVE] = 140;
    servos_change = true;
}

void dead_up() { //! dead values untuned
    servos_angle[Servos::DEAD] = 40;
    servos_change = true;
}

void dead_down() {
    servos_angle[Servos::DEAD] = 140;
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


//* EVAC FUNCTIONS

bool ball_present() {
    //+45 for the diff sensors' offset physically
    return ((l0x_readings[L0X::FRONT] - l1x_readings[L1X::FRONT_BOTTOM]) > 30+45 && l1x_readings[L1X::FRONT_BOTTOM] < 95);
}

//* MISC FUNCTIONS

double pickMotorDist(double prev_rotation) { 
    if (prev_rotation > 0) return MotorL.getDist();
    else return MotorR.getDist()
}

//* ------------------------------------------- DEBUG LOOPS -------------------------------------------

#if loop_movetime
void loop()
{
    if (!digitalRead(SWTPIN)) {
        if (millis() - startDistanceMillis < 1500) {
            Robawt.setSteer(50, 1);
        } else {
            Robawt.setSteer(0, 0);
            Robawt.reset();
        }
    } else {
        Robawt.setSteer(0, 0);
        Robawt.reset();
        startDistanceMillis = millis();
    }
}
#endif

#if loop_movedistance
//~ This is set to double green right now
void loop()
{
    if (!digitalRead(SWTPIN)) {
        double distTravelled = abs(MotorL.getDist()-startDistanceValL) + abs(MotorR.getDist()-startDistanceValR);
        if (distTravelled < 65) {
            Robawt.setSteer(50, 1);
        } else {
            Robawt.setSteer(0, 0);
            Robawt.resetPID();
        }
        Serial.print("Motor L: ");
        Serial.print(MotorL.getDist() - startDistanceValL);
        Serial.print(" || Motor R: ");
        Serial.print(MotorR.getDist() - startDistanceValR);
        Serial.print(" || Distance travelled: ");
        Serial.print(distTravelled);
        Serial.println();
    } else {
        startDistanceValL = MotorL.getDist();
        startDistanceValR = MotorR.getDist();
        Robawt.setSteer(0, 0);
        Robawt.reset();
    }
}
#endif

#if loop_pickball || loop_pickcube
void loop()
{
    if (servos_change) {
        for (int i = servos_start; i != (servos_stop+1); i++) {
        servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
        }
        servos_change = false;
    }

    if (!digitalRead(SWTPIN)) claw_service_up();
    else claw_down();

    for (int i = l0x_start; i != (l0x_stop+1); i++) 
    {
        tcaselect(l0x_pins[i]);
        if(lidarsl0x[i].available()) {
            l0x_readings[i] = lidarsl0x[i].readRangeMillimeters();
        }
    }
    for (int i = l1x_start; i != (l1x_stop+1); i++) 
    {
        tcaselect(l1x_pins[i]);
        if(lidarsl1x[i].dataReady()) {
            l1x_readings[i] = lidarsl1x[i].read(false);
        }
    }

    Serial.print("See ball: ");
    Serial.println(ball_present());

    if (ball_present() || pickupState != 0) {

        switch (pickupState)
        {
            case 0:
                pickupStateTimer = millis();
                pickupState ++;
                break;

            case 1:
                #if loop_pickcube
                claw_close_cube();
                #endif
                #if loop_pickball
                // claw_down();
                claw_close();
                #endif
                if (millis() - pickupStateTimer > 1000) {
                    pickupStateTimer = millis();
                    pickupState ++; }
                break;

            case 2:
                // claw_close();
                claw_up();
                if (millis() - pickupStateTimer > 1000) {
                    pickupStateTimer = millis();
                    pickupState ++; }
                break;

            case 3: 
                claw_open();
                // claw_up();
                if (millis() - pickupStateTimer > 1000) {
                    pickupStateTimer = millis();
                    pickupState ++; }
                break;

            case 4:
                claw_down();
                // claw_open();
                if (millis() - pickupStateTimer > 1000) {
                    pickupStateTimer = millis();
                    pickupState = 0; }
                break;
        }

        Serial.println(pickupState);
    } else {
        sort_alive();
        claw_open();
        claw_down();
    }
}
#endif

#if loop_depositalive || loop_depositdead
void loop()
{
    if (servos_change) {
        for (int i = servos_start; i != (servos_stop+1); i++) {
        servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
        }
        servos_change = false;
    }

    if (!digitalRead(SWTPIN)) {
        #if loop_depositalive
        alive_up();
        #endif
        #if loop_depositdead
        dead_up();
        #endif
        sort_neutral();
    } else {
        #if loop_depositalive
        alive_down();
        sort_alive();
        #endif
        #if loop_depositdead
        dead_down();
        sort_dead();
        #endif
    }
}
#endif