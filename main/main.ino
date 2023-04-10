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
#define debug_distance 0
//^ Runs without normal code
#define loop_movetime 0
#define loop_movedistance 0
#define loop_pickball 0
#define loop_pickcube 0
#define loop_deposit 0
#define isthishardwarestop 0
//^ Force events
#define debug_evac 0
#define debug_evac_exit 0

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
double servos_angle[6] = {0, 140, 180, 0, 130, 90}; //basic states initialised
const double servos_max_angle[6] = {180, 180, 180, 300, 300, 300};
const int servos_pin[6] = {27, 26, 21, 20, 2, 22};
bool servos_change = false;
namespace Servos {
    enum Servos { DEAD, ALIVE, ARM, LEFT, RIGHT, SORT};
}

// double servos_angle[6] = {180, 0, 135, 90, 140, 0}; //basic states initialised
// const double servos_max_angle[6] = {180, 300, 300, 300, 180, 180};
// const int servos_pin[6] = {27, 26, 22, 2, 21, 20};
// bool servos_change = false;
// namespace Servos {
//   enum Servos { ARM, LEFT, RIGHT, SORT, ALIVE, DEAD };
// }

//^ Debugging servos
const int servos_start = Servos::DEAD, //first servo
    servos_stop = Servos::SORT; //last servo

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
int GSState;

//^ Red
long startRedMillis;

//^ Movement and logic
double rotation = 0,
    rpm = 40;
int serialState = 0,
    task = 0, 
    prev_task = 0;
#if debug_evac
int curr = 51; //! force evac
#elif debug_evac_exit
int curr = 90;
#else
int curr = 0;
#endif

//^ Blue cube
double prev_kit_rotation,
    kitStartDist,
    kitBeforeStraightDist,
    kitDistToReverse = 0,
    kitStartReverseDist,
    kitStartTurnBackDist,
    kitTurnBackDist,
    kit_distToTurn;
long endBlueMillis = millis(),
    pickupKitStateTimer;
int pickupKitState;
int afterKitState;

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

//^ Linegap
int linegapState = 0;
double linegapStartDist,
    linegapTurnLeftDist,
    linegapTurnRightDist,
    linegapStartReverse,
    linegapSilverDist;
float linegap_rotation = 0;
long linegap_millis,
    linegapSilverMillis;
bool endLineGap = false;
int debugLinetrackOrigCurr = 0;

//^ Evac
#if debug_evac
bool in_evac = true;
#else
bool in_evac = false;
#endif
long pickupStateTimer,
    startEvacMillis, 
    evac_settime,
    startTurnEvacToLtMillis;
int pickupState = 0,
    OORTurnState,
    wallTurnState,
    wallGapTurnState;
double evac_setdist,
    wall_rot,
    startWallGapDistL,
    startWallGapDistR,
    moveWallGapDist,
    startTurnWallDistL,
    startTurnWallDistR,
    turnWallDist,
    startTurnOORDistL,
    startTurnOORDistR,
    turnOORDist,
    startReverseOORDist,
    startStraightOORDist,
    startTurnEvacToLtDist;
int afterPickupState,
    afterTurnEvacState = 60,
    pickType;
int distFromWallFront,
    prevDistFromWallFront;
int ballType;
double evac_rpm = 40;
bool silverStrip = false;
bool foundLine = false;

//^ Deposit
int depositState = 0,
    depositType = 1,
    afterDepositState;
long depositStateTimer;
double r_k_p = 0.005;
double s_k_p = 0.0015;
int diff, f, s_l, s_r;
double cen_r, cen_s;
double startReverseDistAfterDepL;

//^ Comms
long lastSerialPiSend;

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

    digitalWrite(LEDPIN, LOW);

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

#if !loop_movetime && !loop_movedistance && !loop_pickball && !loop_pickcube && !loop_deposit && !isthishardwarestop
void loop() 
{

    //* COMMUNICATION UPDATES
    //  teensyEvent();
    serialEvent();
    // digitalWrite(LEDPIN, HIGH);

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

        //TODO: clean up the cases here and all the conditions for entering them

        // if (curr == 50 || curr == 51 || curr == 11 || (curr >= 23 && curr <= 26)){ //^ if not evac or not in linegap or not in rescuekit
        //     curr = curr;
        // } else {
        switch (task) 
        {

            //* LINETRACK HANDLING

            case 0: //^ empty linetrack
                //~ if not in obstacle or on red line or not post blue or not current blue or not picking up stuff or not linegap-sweeping or not finding line after evac
                //~ break if curr is not 3, 4, 21 or 22 or above 90
                if ((curr <= 2 || curr >= 5) && curr != 21 && curr != 22 && curr <= 90) { break; } //^ written by xel
                // if (curr == 1) { 
                //     //~ enter post left green mode after minimum turn time
                //     if (millis() - startGSMillis > 200) { 
                //         curr = 21; 
                //         lostGSMillis = millis(); }
                // } else if (curr == 2) {
                //     //~ enter post right green mode after minimum turn time
                //     if (millis() - startGSMillis > 200) { 
                //         curr = 22; 
                //         lostGSMillis = millis(); }
                // } else if (curr == 3) {
                if (curr == 3) {
                    //~ turn a full 180 deg for double green (dist tuned for 50 rpm, 1 rot)
                    distDoubleGSTravelled = abs(MotorL.getDist()-startGSDistL) + abs(MotorR.getDist()-startGSDistR);
                    if (distDoubleGSTravelled > 65) { curr = 0; }
                } else if (curr == 4) {
                    //~ drive forward till over red line
                    if (millis() - startRedMillis > 1000) { curr = 100; }
                // } else if (curr == 5 || curr == 6) { 
                //     //~ only return back to linetrack after 90 if speific time has passed
                //     if (millis() - start90Millis > 500) { curr = 0; }
                } else if (curr == 21 || curr == 22) {
                    //~ post green continue to rotate
                    if (millis() - lostGSMillis > 300) { curr = 0; }
                // } else  { //~ should never happen???
                //     curr = 0; 
                } else if (curr > 90) {
                    foundLine = true;
                }
                break;

            case 1: //^ left green
                if (curr == 3) { break; } //~ double green
                if (curr > 29 && curr < 34) { break; } //~obstacle cases
                if (curr == 0) { 
                    startGSMillis = millis();
                    curr = 1; 
                    startGSDistR = pickMotorDist(-1);
                    GSState = 0; }
                break;

            case 2: //^ right green
                if (curr == 3) { break; } //~ double green
                if (curr > 29 && curr < 34) { break; } //~obstacle cases
                if (curr == 0) { 
                    startGSMillis = millis(); 
                    curr = 2; 
                    startGSDistL = pickMotorDist(1); 
                    GSState = 0; }
                break;

            case 3: //^ double green
                // if (curr > 29 && curr < 34) { break; } //~obstacle cases
                if (curr == 0) { 
                    startGSDistL = MotorL.getDist();
                    startGSDistR = MotorR.getDist(); }
                if (0 <= curr <= 2) { curr = 3; }
                break;
            
            case 4: //^ red line --> go
                if (curr == 0) { startRedMillis = millis(); 
                    curr = 4;}
                break;

            // case 5: //^ left 90 (not in use currently)
            //     if (in_evac) { break; }
            //     if (curr == 6) { break; }
            //     if (curr == 0) { start90Millis = millis(); }
            //     curr = 5;
            //     break;  

            // case 6: //^ right 90 (not in use currently)
            //     if (in_evac) { break; }
            //     if (curr == 5) { break; }
            //     if (curr == 0) { start90Millis = millis(); }
            //     curr = 6;
            //     break;  

            case 7: //^ turning to blue
                if (curr == 0 || (curr == 11 && linegapState == 6)) { 
                    prev_kit_rotation = rotation; 
                    kitStartDist = pickMotorDist(prev_kit_rotation); 
                    afterKitState = curr;
                    curr = 7; }
                break;

            case 8: //^ blue centred 
                if (curr == 7) { curr = 8; }
                break;

            case 11: //^ linegap sweeping
                // if (curr == 1 || curr == 2 || curr == 3) { break; }
                if (curr == 0){
                    linegapStartDist = pickMotorDist(-1);
                    linegapState = 0;
                    curr = 11;
                }
                break;

            // case 12: //^ silver seen
            //     if (curr == 11 && linegapState == 4) { silverStrip = true; }
            //     break;

            //* EVAC HANDLING

            case 20: //^ no ball --> wall track
                if (curr != 60 && curr != 61) { break; }
                if (millis() - startEvacMillis > 240000) { curr = 71; } //finished evac
                else { curr = 60; }
                break;

            case 21: //^ ball
                if (curr != 60 && curr != 61) { break; }
                if (millis() - startEvacMillis > 240000) { curr = 71; } //finished evac
                else { curr = 61; }
                // curr = 61;
                break;

            case 22: //^ deposit alive
                if (curr < 62 || curr > 72) { break; }
                curr = 72;
                depositType = 1; 
                break;

            case 23: //^ deposit dead
                if (curr < 62 || curr > 72) { break; }
                curr = 72;
                depositType = 0; 
                break;

            case 24: //^ finding deposit
                if (curr < 62 || curr > 72) { break; }
                curr = 71;
                break;

            case 25:
                foundLine = false;
                break;

        }   
        // }

        #if debug_curr
        Serial.print("Curr: ");
        Serial.println(curr);
        Serial.print("Task: ");
        Serial.println(task);
        // Serial.print("DebugLG: ");
        // Serial.println(debugLinetrackOrigCurr);
        // Serial.print("KitPickupstate: ");
        // Serial.println(pickupKitState);
        #endif

        //* CURRENT ACTION HANDLED
        switch (curr)
        {

            //* LINETRACK CASES

            case 0: //^ empty linetrack
                send_pi(0);
                Robawt.setSteer(rpm, rotation);
                //~ Trigger obstacle
                if (l1x_readings[L1X::FRONT_BOTTOM] < 85 && l0x_readings[L0X::FRONT] < 40 && (millis() - endBlueMillis > 2000)) { //avoid triggering obstacle cuz l0x not very accurate
                    curr = 30;
                    turn_dir = l0x_readings[L0X::LEFT] > l0x_readings[L0X::RIGHT] ? -1 : 1;
                    obstDist = MotorL.getDist();
                    see_line = false;
                }
                break;

            case 1: //^ left green
                send_pi(0);
                #if debug_led
                led_on = true;
                #endif
                // Robawt.setSteer(rpm, -0.6);
                switch (GSState)
                {
                    case 0:
                        Robawt.setSteer(rpm, 0);
                        if (fabs(pickMotorDist(-1) - startGSDistR) > 5) {
                            GSState++;
                            startGSDistR = pickMotorDist(-1);
                        }
                        break;

                    case 1:
                        Robawt.setSteer(rpm, -0.6);
                        if (fabs(pickMotorDist(-1) - startGSDistR) > 20) {
                            curr = 0;
                            GSState = 0;
                            startGSDistR = pickMotorDist(-1);
                        }
                        break;
                }
                break;

            case 2: //^ right green
                send_pi(0);
                #if debug_led
                led_on = true;
                #endif
                // Robawt.setSteer(rpm, 0.6);
                switch (GSState)
                {
                    case 0:
                        Robawt.setSteer(rpm, 0);
                        if (fabs(pickMotorDist(1) - startGSDistL) > 5) {
                            GSState++;
                            startGSDistL = pickMotorDist(1);
                        }
                        break;

                    case 1:
                        Robawt.setSteer(rpm, 0.6);
                        if (fabs(pickMotorDist(1) - startGSDistL) > 20) {
                            curr = 0;
                            GSState = 0;
                            startGSDistL = pickMotorDist(1);
                        }
                        break;
                }
                break;
            
            case 3: //^ double green
                send_pi(0);
                Robawt.setSteer(50, 1);
                break;

            case 4: //^ red --> go
                send_pi(0);
                #if debug_led
                led_on = true;
                #endif
                Robawt.setSteer(30, 0);
                break;

            // case 5: //^ turn left 90 (not in use currently)
            //     send_pi(0);
            //     Robawt.setSteer(rpm, -1);
            //     #if debug_led
            //     led_on = true;
            //     #endif
            //     break;

            // case 6: //^ turn right 90 (not in use currently)
            //     send_pi(0);
            //     Robawt.setSteer(rpm, 1);
            //     #if debug_led
            //     led_on = true;
            //     #endif
            //     break;

            case 7: //^ turning to blue cube
                send_pi(0);
                Robawt.setSteer(rpm, rotation);
                claw_down();
                claw_halfclose();
                kitBeforeStraightDist = pickMotorDist(prev_kit_rotation); // chooses between L or R motor encoder vals based on previous rotation
                if (ball_present()) {
                    pickupKitState = 0;
                    curr = 26; } //pick up le cube
                break;

            case 8: //^ blue centred
                send_pi(0);
                Robawt.setSteer(rpm, 0);
                claw_down();
                claw_halfclose();
                if (ball_present()) { 
                    pickupKitState = 0;
                    Robawt.setSteer(0, 0);
                    Robawt.resetPID();
                    curr = 26; } 
                break;
            
            //* LINEGAP CASE

            case 11: //^ linegap sweeping (considering reversing for like 1cm before sweeping?)
                send_pi(0);
                Serial.print("linegapState: ");
                Serial.print(linegapState);
                Serial.print("|| LinegapStartDist: ");
                Serial.print(linegapStartDist);
                Serial.print("|| LinegapTurnLeftDist: ");
                Serial.print(linegapTurnLeftDist);
                Serial.print("|| LinegapTurnRightDist: ");
                Serial.println(linegapTurnRightDist);
                Serial.print("|| LinegapRotation: ");
                Serial.println(linegap_rotation);
                if (linegapState == 6) {Serial.print("|| TimeElapsed: "); Serial.println(millis() - linegap_millis);}
                switch(linegapState){
                    
                    case 0: //^scan left first
                        Robawt.setSteer(30, -1);
                        linegapTurnLeftDist = fabs(pickMotorDist(-1) - linegapStartDist);
                        if (endLineGap) {
                            linegap_rotation = -1; 
                            linegapStartReverse = pickMotorDist(-1);
                            endLineGap = false; 
                            linegapState++;
                        }
                        else if (linegapTurnLeftDist >= 19) { //^ if bot has turned 90 degs left
                            linegapStartReverse = pickMotorDist(-1);
                            // linegap_rotation = 0;
                            linegapState ++;
                        }
                        break;

                    case 1: //^ turn back to original pos
                        Robawt.setSteer(30, 1);
                        if (fabs(pickMotorDist(-1) - linegapStartReverse) >= linegapTurnLeftDist) {
                            linegapStartDist = pickMotorDist(1);
                            linegapState ++;
                        }
                        break;

                    case 2: //^ scan right
                        Robawt.setSteer(30, 1); 
                        linegapTurnRightDist = fabs(pickMotorDist(1) - linegapStartDist);
                        if (endLineGap) {
                            linegap_rotation = 1;
                            linegap_millis = millis();
                            endLineGap = false;
                            linegapState = 6;
                        }
                        else if (linegapTurnRightDist >= linegapTurnLeftDist){ //^ turns right to match left dist
                            linegapStartReverse = pickMotorDist(1);
                            linegapState ++;
                        }
                        break;

                    case 3: //^ return to original pos
                        Robawt.setSteer(30, -1);
                        if (fabs(pickMotorDist(1) - linegapStartReverse) > linegapTurnRightDist) {
                            if (linegap_rotation == -1) { 
                                linegap_millis = millis();
                                linegapState = 6;
                            } else {
                                linegap_rotation = 0;
                                linegapState ++;
                                linegapSilverDist = pickMotorDist(-1);
                                linegapSilverMillis = millis();
                            }
                        }
                        
                        break;

                    case 4: //^ actual gap or silver tape?
                        digitalWrite(LEDPIN, HIGH);
                        //~ Moving back
                        // if (fabs(pickMotorDist(-1) - linegapSilverDist) > 4) { //^ move back a few cm to expand FOV
                        //     Robawt.setSteer(0, 0);
                        //     endLineGap = false;
                        //     linegapSilverDist = pickMotorDist(-1);
                        //     linegapState ++;
                        //     digitalWrite(LEDPIN, LOW);
                        // } else {
                        //     Robawt.setSteer(-30, 0);
                        // }
                        //~ Wait
                        Robawt.setSteer(0, 0);
                        if (millis() - linegapSilverMillis > 1000) {
                            endLineGap = false;
                            linegapSilverDist = pickMotorDist(-1);
                            linegapState ++;
                            digitalWrite(LEDPIN, LOW);
                        }
                        if (silverStrip) { curr = 51; } //^ jump to evac
                        break;

                    case 5: //^ move back to original pos
                        //~ Moving back forwards
                        // Robawt.setSteer(30, 0);
                        // if (fabs(pickMotorDist(-1) - linegapSilverDist) > 7) { 
                        //     endLineGap = false;
                        //     linegap_millis = millis();
                        //     linegapState ++;
                        // }
                        //~ Wait
                        linegapState ++ ;
                        linegap_millis = millis();
                        endLineGap = false;
                        break;
                     
                    case 6: //^ turn to the desired direction
                        if (endLineGap){ 
                            curr = 0;
                            linegapState = 0;
                            endLineGap = false;
                            linegap_rotation = 0;
                        } else if (linegap_rotation == 0) {
                            Robawt.setSteer(rpm, rotation);
                        } else if (millis() - linegap_millis > 2000) {
                            Robawt.setSteer(-30, 0);
                            Serial.println("REVERSING 倒车， 倒车");
                        } else {
                            Robawt.setSteer(30, linegap_rotation);
                        }
                        break; 

                        /*
                        if (endLineGap){ 
                            curr = 0;
                            linegapState = 0;
                            endLineGap = false;
                            linegap_rotation = 0;
                        }
                        else if (linegap_rotation == 0) {
                            Robawt.setSteer(30, rotation);
                        }   
                        else if (millis() - linegap_millis > 1000) { 
                            Robawt.setSteer(-30, 0);
                        } else {
                            Robawt.setSteer(30, linegap_rotation);
                        }
                        */
                        // break; 
                }
                break;

            //* POST CASES

            case 21: //^ post left green
                send_pi(0);
                Robawt.setSteer(rpm, -0.6);
                break;

            case 22: //^ post right green
                send_pi(0);
                Robawt.setSteer(rpm, 0.6);
                break;

            case 23: //^ post cube - initialisating vars
                send_pi(0);
                #if debug_led
                led_on = true;
                #endif
                kitStartReverseDist = pickMotorDist(prev_kit_rotation);
                kitDistToReverse = pickMotorDist(prev_kit_rotation) - kitBeforeStraightDist;
                curr = 24;
                break;

            case 24: //^ post cube reversing
                send_pi(0);
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
                send_pi(0);
                #if debug_led
                led_on = true;
                #endif
                Robawt.setSteer(-rpm, prev_kit_rotation);
                kitTurnBackDist =  abs(pickMotorDist(prev_kit_rotation)-kitStartTurnBackDist);
                if (kitTurnBackDist > kit_distToTurn) { 
                    endBlueMillis = millis();
                    curr = afterKitState; }
                break;

            case 26: //^ pickup for rescue kit
                send_pi(0);
                switch (pickupKitState)
                {
                    case 0:
                        pickupKitStateTimer = millis();
                        pickupKitState ++;
                        sort_dead();
                        break;

                    case 1:
                        claw_close_cube();
                        if (millis() - pickupKitStateTimer > 1000) {
                            pickupKitStateTimer = millis();
                            pickupKitState ++; }
                        break;

                    case 2:
                        claw_up();
                        if (millis() - pickupKitStateTimer > 1000) {
                            pickupKitStateTimer = millis();
                            pickupKitState ++; }
                        break;

                    case 3: 
                        claw_open();
                        if (millis() - pickupKitStateTimer > 1000) {
                            pickupKitStateTimer = millis();
                            pickupKitState ++; }
                        break;

                    case 4:
                        claw_down();
                        if (millis() - pickupKitStateTimer > 1000) {
                            pickupKitStateTimer = millis();
                            pickupKitState = 0; 
                            curr = 23; }
                        break;
                }
                Robawt.setSteer(0, 0);
                Robawt.resetPID();
                break;

            //* OBSTACLE LOGIC (30 - 33)

            case 30: //^ reversing after detecting obstacle
                send_pi(0);
                Robawt.setSteer(-40, 0);
                Serial.println(MotorL.getDist() - obstDist);
                if (fabs(MotorL.getDist() - obstDist) > 7.5) {
                    obstState = 0;
                    sideObstDist = turn_dir == 1 ? &l0x_readings[L0X::LEFT] : &l0x_readings[L0X::RIGHT]; //^ getting address of readings to constantly update the val
                    curr = 31; }
                break;

            case 31: //^ turning 90 degrees
                if (turn_dir == 1) { send_pi(6); }
                else { send_pi(5); }
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
                if (turn_dir == 1) { send_pi(6); }
                else { send_pi(5); }
                switch (obstState)
                {
                    case 0:
                        Robawt.setSteer (40, 0);
                        if (*sideObstDist < 150) { obstState++; }
                        break;
                        
                    case 1:
                        Robawt.setSteer (40, 0);
                        if (*sideObstDist > 150) { obstState++; }
                        break;

                    case 2:
                        Robawt.setSteer (40, -o_rotation*turn_dir);
                        if (*sideObstDist < 150) { obstState++; }
                        break;

                    case 3:
                        Robawt.setSteer (40, -o_rotation*turn_dir);
                        if (*sideObstDist > 150) { obstState = 0; }
                        break;
                }
                Serial.print("Obstacle state: ");
                Serial.println(obstState);
                //~ Minimum obstacle turn time
                if (see_line && (millis() - obst_time_start) > 200){
                    curr = 33;
                    obst_time_start = millis();
                    obstState = 0; 
                    obstStartTurnBackDist = pickMotorDist(turn_dir); }
                break;

            case 33: //^ turning back to line after obstacle
                send_pi(0);
                Robawt.setSteer(30, turn_dir*0.7);
                obstCurrDist = pickMotorDist(turn_dir);
                if (obstCurrDist - obstStartTurnBackDist > 25) { curr = 0; }
                break;

            //* EVAC CASES

            case 50: //^ pickup
                if (afterPickupState != 71) { send_pi(1); }
                switch (pickupState)
                {
                    case 0:
                        pickupStateTimer = millis();
                        pickupState ++;
                        break;

                    case 1:
                        claw_close();
                        if (ballType == 1) { sort_alive(); }
                        else if (ballType == 0) { sort_dead(); }
                        if (millis() - pickupStateTimer > 1000) {
                            pickupStateTimer = millis();
                            pickupState ++; 
                            Robawt.setSteer(0, 0);
                            Robawt.resetPID(); }
                        break;

                    case 2:
                        if (ballType == 1) { sort_alive(); }
                        else if (ballType == 0) { sort_dead(); }
                        Robawt.setSteer(-30, 0);
                        if (millis() - pickupStateTimer > 1000) {
                            pickupStateTimer = millis();
                            pickupState ++; }
                        break;
                        
                    case 3:
                        claw_up();
                        Robawt.setSteer(0, 0);
                        Robawt.resetPID();
                        if (ballType == 1) { sort_alive(); }
                        else if (ballType == 0) { sort_dead(); }
                        if (millis() - pickupStateTimer > 1000) {
                            pickupStateTimer = millis();
                            pickupState ++; }
                        break;

                    case 4: 
                        claw_open();
                        Robawt.setSteer(0, 0);
                        Robawt.resetPID();
                        if (ballType == 1) { sort_alive(); }
                        else if (ballType == 0) { sort_dead(); }
                        if (millis() - pickupStateTimer > 1000) {
                            pickupStateTimer = millis();
                            pickupState ++; }
                        break;

                    case 5:
                        claw_down();
                        sort_neutral();
                        Robawt.setSteer(0, 0);
                        Robawt.resetPID();
                        if (millis() - pickupStateTimer > 1000) {
                            pickupStateTimer = millis();
                            pickupState = 0; 
                            curr = afterPickupState; }
                        break;
                }
                break;

            case 51: //^ evac initialisation
                send_pi(1);
                startEvacMillis = millis();
                curr = 60;
                in_evac = true;
                pickType = 1;
                lastSerialPiSend = millis();
                break;

            case 60: //^ no ball walltrack
                send_pi(1);
                claw_open();
                evac_settime = millis() - (startEvacMillis+30000);
                if (evac_settime < 0) {evac_settime = 1;
                    #if debug_led
                    led_on = true;
                    #endif
                    }
                evac_setdist = 120 + (evac_settime/250);
                if (evac_setdist > 600) {evac_setdist = 600;}
                wall_rot = (evac_setdist - l0x_readings[L0X::FRONT_LEFT]) * 0.0095;
                Robawt.setSteer(evac_rpm, wall_rot);
                Serial.print("wall rotation");
                Serial.println(wall_rot);
                if (ball_present()) { 
                    curr = 50;
                    pickupState = 0;
                    afterPickupState = 60; }
                else if (OOR_present()) {
                    startReverseOORDist = MotorL.getDist();
                    curr = 65;
                    OORTurnState = 0; 
                    afterTurnEvacState = 60; }
                else if (wall_present()) {
                    startReverseOORDist = MotorL.getDist();
                    curr = 66;
                    wallTurnState = 0; 
                    afterTurnEvacState = 60; }
                else if (wallgap_present()) { 
                    curr = 67;
                    wallGapTurnState = 0;
                    startWallGapDistL = MotorL.getDist();
                    startWallGapDistR = MotorR.getDist(); 
                    afterTurnEvacState = 60; }
                break;

            case 61: //^ ball track
                send_pi(1);
                #if debug_led
                led_on = true;
                #endif
                if (rotation < 0.5) { claw_halfclose(); }
                Robawt.setSteer(evac_rpm, rotation);
                if (ball_present()) { 
                    curr = 50;
                    afterPickupState = 60; }
                else if (OOR_present()) {
                    startReverseOORDist = MotorL.getDist();
                    curr = 65;
                    OORTurnState = 0; 
                    afterTurnEvacState = 61; }
                else if (wall_present()) {
                    startReverseOORDist = MotorL.getDist();
                    curr = 66;
                    wallTurnState = 0; 
                    afterTurnEvacState = 61; }
                else if (wallgap_present()) { 
                    curr = 67;
                    wallGapTurnState = 0;
                    startWallGapDistL = MotorL.getDist();
                    startWallGapDistR = MotorR.getDist(); 
                    afterTurnEvacState = 61; }
                break;
                
            case 65: //^ reverse then turn right then go straight when out of range
                switch (OORTurnState)
                {
                    case 0:
                        Robawt.setSteer(-30, 0);
                        if (fabs(MotorL.getDist() - startReverseOORDist) > 20) { 
                            OORTurnState ++; 
                            startTurnOORDistL = MotorL.getDist();
                            startTurnOORDistR = MotorR.getDist(); }
                        break;

                    case 1:
                        Robawt.setSteer(30, 1);
                        turnOORDist = abs(MotorL.getDist()-startTurnOORDistL) + abs(MotorR.getDist()-startTurnOORDistR);
                        if (turnOORDist > 35) { 
                            OORTurnState ++;
                            startStraightOORDist = MotorL.getDist(); }
                        break;

                    case 2:
                        Robawt.setSteer(30, 0);
                        if (fabs(MotorL.getDist() - startStraightOORDist) > 4) {
                            curr = afterTurnEvacState; 
                            OORTurnState = 0; }
                        break;
                }
                break;

            case 66: //^ turn when stuck on wall
                switch (wallTurnState)
                {
                    case 0:
                        Robawt.setSteer(-30, 0);
                        if (fabs(MotorL.getDist() - startReverseOORDist) > 10) { 
                            wallTurnState ++; 
                            startTurnWallDistL = MotorL.getDist();
                            startTurnWallDistR = MotorR.getDist(); }
                        break;

                    case 1:
                        Robawt.setSteer(30, 1);
                        turnWallDist = abs(MotorL.getDist()-startTurnWallDistL) + abs(MotorR.getDist()-startTurnWallDistR);
                        if (turnWallDist > 35) { 
                            wallTurnState = 0;
                            curr = afterTurnEvacState;  }
                        break;
                }
                Serial.print("Wall turn state: ");
                Serial.println(wallTurnState);
                break;

            case 67: //^ turn when gap is on left |____   ___|
                switch (wallGapTurnState)
                {
                    case 0:
                        Robawt.setSteer(30, 0.15);
                        moveWallGapDist = abs(MotorL.getDist()-startWallGapDistL) + abs(MotorR.getDist()-startWallGapDistR);
                        if (wall_present()) {
                            startReverseOORDist = MotorL.getDist();
                            curr = 66;
                            wallTurnState = 0; }
                        else if (ball_present()) {
                            curr = 50;
                            pickupState = 0; 
                            afterPickupState = 67; }
                        else if (moveWallGapDist > 25) { 
                            curr = afterTurnEvacState; 
                            wallGapTurnState = 0; }
                }
                break;

            // case 69: //^ centering
                // if (front_see_infinity()) { f =  600; }
                // else { f = l0x_readings[L0X::FRONT]; }
                // if (right_see_infinity()) { s_r =  600; }
                // else { s_r = l0x_readings[L0X::RIGHT]; }
                // if (left_see_infinity()) { s_l =  600; }
                // else { s_l = l0x_readings[L0X::LEFT]; }
                // diff = l0x_readings[L0X::RIGHT] - l0x_readings[L0X::LEFT];
                // cen_r = constrain(diff * r_k_p, -1, 1);
                // cen_s = constrain(f * s_k_p, 0.001, 1);
                // if (cen_r > 0) { rotation = pow(cen_r, cen_s); }
                // else { rotation = -pow(-cen_r, cen_s); }
                // Robawt.setSteer(60, rotation);
                // Serial.print("cen_r: ");
                // Serial.print(cen_r);
                // Serial.print("  cen_s: ");
                // Serial.print(cen_s);
                // Serial.print("  rotation: ");
                // Serial.println(rotation);
                // if (fabs(l0x_readings[L0X::RIGHT]) <=  600 && fabs(l0x_readings[L0X::LEFT]) <= 600 && fabs(diff) <= 100 && fabs(f) <= 400) {
                //     curr = 70; }
                // break;

            // case 70: //^ turning around for the evac
            //     Robawt.setSteer(50, 1);
            //     break;

            case 71: //^ centering for deposit
            //TODO: check if the ball gets stuck when navigating to deposit code works
                if (depositType == 1) { send_pi(2); }
                else { send_pi(3); }
                #if debug_led
                led_on = true;
                #endif
                // evac_setdist = 140 + (millis() - startEvacMillis)/10; //constant changed to speed up process
                // if (evac_setdist > 600) {evac_setdist = 600;}
                evac_setdist = 600;
                wall_rot = (evac_setdist - l0x_readings[L0X::FRONT_LEFT]) * 0.0095;
                Robawt.setSteer(evac_rpm, wall_rot);
                if (ball_present()) {
                    curr = 50;
                    pickupState = 0;
                    afterPickupState = 71;
                } else if (OOR_present()) {
                    startReverseOORDist = MotorL.getDist();
                    curr = 65;
                    OORTurnState = 0; 
                    afterTurnEvacState = 71; }
                else if (wall_present()) {
                    startReverseOORDist = MotorL.getDist();
                    curr = 66;
                    wallTurnState = 0; 
                    afterTurnEvacState = 71; }
                else if (wallgap_present()) { 
                    curr = 67;
                    wallGapTurnState = 0;
                    startWallGapDistL = MotorL.getDist();
                    startWallGapDistR = MotorR.getDist(); 
                    afterTurnEvacState = 71; }
                break;
                
            case 72: //^ heading to alive zone / dead zone
                if (depositType == 1) { send_pi(2); }
                else { send_pi(3); }
                Robawt.setSteer(40, rotation);
                sort_neutral();
                if (ball_present()) { 
                    curr = 50;
                    afterPickupState = 71; }
                if ((l1x_readings[L1X::FRONT_BOTTOM] < 60 || l0x_readings[L0X::FRONT] < 25) && !ball_present()) { 
                    curr = 80;
                    afterDepositState = 73;
                    depositStateTimer = millis();
                    depositState = 0; }
                break;

            case 73: //^ post deposit
                if (depositType == 1) {  // go deposit dead now
                    depositType = 0; 
                    curr = 74; 
                    send_pi(3); 
                    startReverseDistAfterDepL = pickMotorDist(1); }
                else { curr = 75; 
                    send_pi(4); 
                    startReverseDistAfterDepL = pickMotorDist(1); } // get out of evac
                break;

            case 74: //^ reversing from alive zone
                send_pi(3);
                Robawt.setSteer(-40, 0);
                if (fabs(pickMotorDist(1) - startReverseDistAfterDepL) > 60) {
                    curr = 71;
                    depositType = 0;
                }
                break;

            case 75: //^ after dead zone
                send_pi(4);
                Robawt.setSteer(-40, 0);
                if (fabs(pickMotorDist(1) - startReverseDistAfterDepL) > 60) {
                    curr = 90;
                }
                break;

            case 80: //^ deposit
                Robawt.setSteer(0, 0);
                Robawt.resetPID();
                switch (depositState)
                {
                    case 0:
                        // sort_neutral(); //! add back in after servo attached back
                        if (depositType == 0) { dead_up(); }
                        else { alive_up(); }
                        if (millis() - depositStateTimer > 1500) { 
                            depositStateTimer = millis();
                            depositState ++; }
                        break;

                    case 1:
                        if (depositType == 0) { dead_down(); }
                        else { alive_down(); }
                        if (millis() - depositStateTimer > 1000) { 
                            depositStateTimer = millis();
                            depositState = 0; 
                            curr = afterDepositState; }
                        break;
                }
                break;

            case 90: //^ get out of evac by centering and scanning
                send_pi(4);
                #if debug_led
                led_on = true;
                #endif
                // evac_setdist = 140 + (millis() - startEvacMillis)/10; //constant changed to speed up process
                // if (evac_setdist > 600) {evac_setdist = 600;}
                evac_setdist = 200;
                wall_rot = (evac_setdist - l0x_readings[L0X::FRONT_LEFT]) * 0.0095;
                Robawt.setSteer(evac_rpm, wall_rot);
                if (front_see_infinity() && frontLeft_see_infinity()) {
                    curr = 93; 
                } else if (front_see_infinity()) {
                    curr = 91;
                } else if (frontLeft_see_infinity()) {
                    startTurnEvacToLtDist = pickMotorDist(-1);
                    curr = 92;
                }
                if (ball_present()) {
                    curr = 50;
                    pickupState = 0;
                    afterPickupState = 90;
                }
                // } else if (OOR_present()) {
                //     startReverseOORDist = MotorL.getDist();
                //     curr = 65;
                //     OORTurnState = 0; 
                //     afterTurnEvacState = 71; }
                // else if (wall_present()) {
                //     startReverseOORDist = MotorL.getDist();
                //     curr = 66;
                //     wallTurnState = 0; 
                //     afterTurnEvacState = 71; }
                // else if (wallgap_present()) { 
                //     curr = 67;
                //     wallGapTurnState = 0;
                //     startWallGapDistL = MotorL.getDist();
                //     startWallGapDistR = MotorR.getDist(); 
                //     afterTurnEvacState = 71; }
                break;

            case 91: //^ checking if line if facing OOR
            //TODO: add timeout for this case
                send_pi(4);
                Robawt.setSteer(40, 0);
                if (OOR_present && foundLine) { 
                    startTurnEvacToLtMillis = millis();
                    curr = 96; }
                break;

            case 92: //^ checking if line when left is OOR
                send_pi(4);
                if (pickMotorDist(-1) - startTurnEvacToLtDist < 35) {
                    Robawt.setSteer(40, -0.5);
                } else {
                    Robawt.setSteer(40, 0);
                }
                if (OOR_present && foundLine) { 
                    startTurnEvacToLtMillis = millis();
                    curr = 96; }
                break;

            case 93:
                break;

            case 96: //^ turn to line
                send_pi(4);
                Robawt.setSteer(rpm, rotation);
                if (millis() - startTurnEvacToLtMillis > 1000) { curr = 0; }
                break;

            //* STOP

            case 100: //^ red --> stop
                send_pi(0);
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
        
        #if debug_evac
        curr = 51; //! force evac
        depositType = 1;
        #elif debug_evac_exit
        curr = 90;
        foundLine = false;
        #else
        curr = 0;
        linegapState = 0;
        obstState = 0;
        #endif

        claw_down();
        claw_open();
        sort_neutral();
        send_pi(9);
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
        if (serialData == 255 || serialData == 254 || serialData == 253 || serialData == 252 || serialData == 251 || serialData == 250 || serialData == 249) {
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
                case 251:
                    endLineGap = (bool)serialData;
                    break;
                case 250:
                    // if (curr == 61) { 
                        ballType = (int)serialData; 
                        // }
                    break;
                case 249:
                    silverStrip = (bool)serialData;
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
    if (millis() - lastSerialPiSend > 100) {
        Serial1.print(i);
        lastSerialPiSend = millis();
    }
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
    servos_angle[Servos::RIGHT] = 130; 
    servos_angle[Servos::LEFT] = 0;
    servos_change = true;
}

void claw_close() { //ball
    servos_angle[Servos::RIGHT] = 15;
    servos_angle[Servos::LEFT] = 115;
    servos_change = true;
}

void claw_close_cube() {
    servos_angle[Servos::RIGHT] = 15;
    servos_angle[Servos::LEFT] = 115;
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
    servos_angle[Servos::ARM] = 180;
    servos_change = true;
}

void alive_up() {
    servos_angle[Servos::ALIVE] = 80;
    servos_change = true;
}

void alive_down() {
    servos_angle[Servos::ALIVE] = 140;
    servos_change = true;
}

void dead_up() { //! dead values untuned
    servos_angle[Servos::DEAD] = 30;
    servos_change = true;
}

void dead_down() {
    servos_angle[Servos::DEAD] = 0;
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
    return ((l0x_readings[L0X::FRONT]+45 - l1x_readings[L1X::FRONT_BOTTOM]) > 30 && l1x_readings[L1X::FRONT_BOTTOM] < 80);
}

bool wall_present() {
    return (l0x_readings[L0X::FRONT]+45 < 90 && l1x_readings[L1X::FRONT_BOTTOM] < 90);
}

bool OOR_present() {
    return ((front_see_infinity() && (l0x_readings[L0X::RIGHT] < 200 || right_see_infinity())) //if front sees out, and right either sees wall or OOR
    || (front_see_infinity() && (left_see_infinity() || (frontLeft_see_infinity() && l0x_readings[L0X::LEFT] > 180)))); //if left and front sees out
}

bool wallgap_present() {
    return (l0x_readings[L0X::LEFT] > 1680 || l0x_readings[L0X::FRONT_LEFT] > 1680);
}

bool front_see_infinity() {
    return (l0x_readings[L0X::FRONT] > 1680);
}

bool left_see_infinity() {
    return  (l0x_readings[L0X::LEFT] > 1680);
}

bool right_see_infinity() {
    return  (l0x_readings[L0X::RIGHT] > 1680);
}

bool frontLeft_see_infinity() {
    return (l0x_readings[L0X::FRONT_LEFT] > 1680);
}

//* MISC FUNCTIONS

double pickMotorDist(double prev_rotation) { 
    if (prev_rotation > 0) { return MotorL.getDist(); }
    else { return MotorR.getDist(); }
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
    for (int i = servos_start; i != (servos_stop+1); i++) {
        servos[i].detach();
    }
    if (!digitalRead(SWTPIN)) {
        double distTravelled = fabs(MotorL.getDist()-startDistanceValL) + fabs(MotorR.getDist()-startDistanceValR);
        if (distTravelled < 120) {
            Robawt.setSteer(-40, 0);
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

#if loop_deposit
void loop()
{
    if (servos_change) {
        for (int i = servos_start; i != (servos_stop+1); i++) {
        servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
        }
        servos_change = false;
    }

    if (!digitalRead(SWTPIN)) {
        alive_up();
        dead_up();
        sort_neutral();
    } else {
        alive_down();
        dead_down();
        claw_open();
    }
}
#endif

#if loop_debugsetup
void loop() {}
#endif

#if isthishardwarestop
void loop() {
    if (!digitalRead(SWTPIN)) {
        Robawt.setSteer(30, -0.5); 
    } else { 
        Robawt.setSteer(0, 0); 
        Robawt.resetPID();
    }
}
#endif