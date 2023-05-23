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
#define debug_evac_speedup 1

//^ Runs without normal code
#define loop_movetime 0 
#define loop_movedistance 0
#define loop_pickball 0
#define loop_pickcube 0
#define loop_deposit 0
#define isthishardwarestop 0
//^ Force events
#define debug_evac 0
#define debug_deposit 0
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
int l0x_readings[4] = {200, 0, 0, 0};
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

bool depositedAlready = false;
bool seesaw_now = true;
bool small_linegapend = true;

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
bool piOn = false;
double rotation = 0,
    rpm = 40;
int serialState = 0,
    task = 0, 
    prev_task = 0;
#if debug_evac
int curr = 51; //! force evac
#elif debug_evac_exit
int curr = 90;
#elif debug_deposit
int curr = 71;
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
double linegapTurnLeftDist,
    linegapTurnRightDist,
    linegapSilverDist,
    linegapStartDistL,
    linegapStartDistR,
    linegapStartReverseR,
    linegapStartReverseL,
    LGdistMovedForward;
    

bool seeLeftForEvac = false, 
    seeRightForEvac = false;

float linegap_rotation = 0;
long linegap_millis,
    linegapSilverMillis,
    endLinegapMillis,
    LGLastTriggeredTranslations;
bool endLineGap = false;
int debugLinetrackOrigCurr = 0;

//^ Silver
bool leftSaw,
    rightSaw;

//^ Evac
#if debug_evac || debug_deposit || debug_evac_exit
bool in_evac = true;
#else
bool in_evac = false;
#endif
long pickupStateTimer,
    startEvacMillis, 
    evac_settime,
    startTurnEvacToLtMillis,
    endEvacMillis;
int pickupState = 0,
    OORTurnState,
    wallTurnState,
    wallGapTurnState,
    postRescueKitState,
    enterEvacState = 0;
double evac_setdist,
    evac_startdist,
    k_p_wall_rot,
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
    startTurnEvacToLtDist,
    postRescueKitTurnDist,
    turnedEvacExitDist,
    depositToExitDist,
    startHeadingToDepositDist,
    evac_time_constant;
int afterPickupState,
    afterTurnEvacState = 60,
    pickType,
    evacExitState = 0,
    depositToExitState = 0;
int distFromWallFront,
    prevDistFromWallFront,
    evacExitFrontVal,
    frontLeftForDepositVal;
int ballType;
double evac_rpm = 45,
    evac_exit_rpm = 50,
    evac_deposit_rpm = 65;
bool isWallInFront = false;
bool silverStrip = false;
bool foundLine = false;
bool afterEvac = false;
bool seeWallForEvac = false;
bool leftSawExtensions = false,
    rightSawExtensions = false;

//^ TIME
bool hasFlippedSwitchOnce = false;
long LineTrackStartTime;
// long LineTrackEndTime;
long LineTrackTimeElapsed;

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

bool linetrackleftSaw = false;

bool linetrackrightSaw = false;

long lastSawleftMillis;
long lastSawRightMillis;
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
        if (hasFlippedSwitchOnce == false) {
            LineTrackStartTime = millis(); 
            hasFlippedSwitchOnce = true;
        }
        LineTrackTimeElapsed = millis() - LineTrackStartTime;
        
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
                
                if (curr == 0 && silverStrip && !far_obstacle_present()) { 
                    curr = 51; 
                    enterEvacState = 0;} //^ jump to evac
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
                    //~ drive forward till oaver red line
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
                    endEvacMillis = millis();
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
                if (curr == 0 && afterEvac) { startRedMillis = millis(); 
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
                if (curr == 0 || (curr == 11 && linegapState == 6)) { //! hardcode
                    prev_kit_rotation = rotation; 
                    kitStartDist = pickMotorDist(prev_kit_rotation); 
                    afterKitState = curr;
                    curr = 7; }
                break;

            case 8: //^ blue centred 
                if (curr == 7){ curr = 8; } //! hardcode
                break;

            case 11: //^ linegap sweeping
                // if (curr == 1 || curr == 2 || curr == 3) { break; }
                if (curr == 0 && millis() - endLinegapMillis > 500){
                    if (millis() - LGLastTriggeredTranslations > 4000) { linegapState = -2; }
                    else { linegapState = 0; }  
                    // linegapState = -2;
                    linegapStartDistL = pickMotorDist(-1);
                    linegapStartDistR = pickMotorDist(1);
                    linegap_rotation = 0;
                    curr = 11;
                }
                break;

            // case 12: //^ silver seen
            //     if (curr == 11 && linegapState == 4) { silverStrip = true; }
            //     break;

            //* EVAC HANDLING

            case 20: //^ no ball --> wall track
                if (curr != 60 && curr != 61) { break; }
                if (millis() - startEvacMillis > 125000) { curr = 71; } //finished evac
                else { curr = 60; }
                break;

            case 21: //^ ball
                if (curr != 60 && curr != 61) { break; }
                if (millis() - startEvacMillis > 125000) { curr = 71; } //finished evac
                else { curr = 61; }
                // curr = 61;
                break;

            case 22: //^ deposit alive
                if (curr < 62 || curr > 72) { break; }
                if (curr == 71) startHeadingToDepositDist = pickMotorDist(-1); 
                curr = 72;
                depositType = 1; 
                break;

            case 23: //^ deposit dead
                if (curr < 62 || curr > 72) { break; }
                if (curr == 71) { startHeadingToDepositDist = pickMotorDist(-1); }
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
                if (obstacle_present() && (millis() - endBlueMillis > 2000)) { //! hardcode //avoid triggering obstacle cuz l0x not very accurate
                    curr = 30;
                    // turn_dir = l0x_readings[L0X::LEFT] > l0x_readings[L0X::RIGHT] ? -1 : 1;
                    turn_dir = -1;
                    obstDist = MotorL.getDist();
                    see_line = false;
                }
                //~ Trigger evac
                if (left_see_reallyclosewall() && right_see_closewall() && l0x_readings[L0X::FRONT] < 1300 && (millis() - endEvacMillis) > 2000) {
                    curr = 51;
                }
                if (left_see_reallyclosewall()) {
                    linetrackleftSaw = true;
                    lastSawleftMillis = millis();
                }
                if (right_see_reallyclosewall()) {
                    linetrackrightSaw = true;
                    lastSawRightMillis = millis();
                }
                if (millis() - lastSawleftMillis > 500) {
                    linetrackleftSaw = false;
                }
                if (millis() - lastSawRightMillis > 500) {
                    linetrackrightSaw = false;
                }
                if (linetrackrightSaw && linetrackleftSaw) {
                    curr = 51;
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
                        Robawt.setSteer(rpm, -0.7);
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
                        Robawt.setSteer(rpm, 0.7);
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
                if (obstacle_present()) { 
                    curr = 30;
                    turn_dir = l0x_readings[L0X::LEFT] > l0x_readings[L0X::RIGHT] ? -1 : 1;
                    obstDist = MotorL.getDist();
                    see_line = false;
                }
                if (millis() - startRedMillis > 1000) {
                    curr = 100;
                }
                
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
                else if (obstacle_present()) {
                    curr = 23;
                }
                break;

            case 8: //^ blue centred
                send_pi(0);
                Robawt.setSteer(40, 0);
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
                Serial.print("|| LinegapStartDistL: ");
                Serial.print(linegapStartDistL);
                Serial.print("|| LinegapStartDistL: ");
                Serial.print(linegapStartDistR);
                Serial.print("|| LinegapTurnLeftDist: ");
                Serial.print(linegapTurnLeftDist);
                Serial.print("|| LinegapTurnRightDist: ");
                Serial.println(linegapTurnRightDist);
                Serial.print("|| LinegapRotation: ");
                Serial.println(linegap_rotation);
                if (linegapState == 6) {Serial.print("|| TimeElapsed: "); Serial.println(millis() - linegap_millis);}
                switch(linegapState){
                    case -2:
                        Robawt.setSteer(30, 0);
                        // Robawt.resetPID();
                        if (right_see_reallyclosewall()) {
                            rightSaw = true;
                        } 
                        if (left_see_reallyclosewall())
                        {
                            leftSaw = true;
                        }
                        LGdistMovedForward = fabs(pickMotorDist(-1) - linegapStartDistL);
                        if (LGdistMovedForward > 15) {
                            if (seesaw_now) { linegapState = 8; }
                            else { linegapState++; }
                            linegapStartDistL = pickMotorDist(-1);
                            linegapStartDistR = pickMotorDist(1);
                            LGLastTriggeredTranslations = millis();
                        }
                        if (leftSaw && rightSaw) { 
                            curr = 51;
                        }
                        break;

                    case -1:
                        Robawt.setSteer(-30, 0);
                        if (right_see_reallyclosewall()) {
                            rightSaw = true;
                        } 
                        if (left_see_reallyclosewall())
                        {
                            leftSaw = true;
                        }
                        if (fabs(pickMotorDist(-1) - linegapStartDistL) > LGdistMovedForward) {
                            linegapState++;
                            linegapStartDistL = pickMotorDist(-1);
                            linegapStartDistR = pickMotorDist(1);
                            LGLastTriggeredTranslations = millis();
                        }
                        else if (small_linegapend) {
                            linegapState = 7;
                        }
                        if (leftSaw && rightSaw) { 
                            curr = 51;
                        }
                        break;

                    case 0: //^scan left first
                        rightSaw = false;
                        leftSaw = false;
                        Robawt.setSteer(30, -1);
                        linegapTurnLeftDist = fabs(pickMotorDist(-1) - linegapStartDistL);
                        if (endLineGap) {
                            linegap_rotation = -1; 
                            linegapStartReverseL = pickMotorDist(-1);
                            linegapStartReverseR = pickMotorDist(1);
                            linegapStartDistL = pickMotorDist(-1);
                            linegapStartDistR = pickMotorDist(1);
                            endLineGap = false; 
                            linegapState++;
                        }
                        else if (linegapTurnLeftDist >= 19.3) { //^ if bot has turned 90 degs left
                            linegap_rotation = 0;
                            linegapStartReverseL = pickMotorDist(-1);
                            linegapStartReverseR = pickMotorDist(1);
                            linegapStartDistL = pickMotorDist(-1);
                            linegapStartDistR = pickMotorDist(1);
                            endLineGap = false;
                            linegapState ++;
                        }
                        break;

                    case 1: //^ turn back to original pos
                        Robawt.setSteer(30, 1);
                        if (fabs(pickMotorDist(-1) - linegapStartReverseL) >= linegapTurnLeftDist) {
                            linegapStartReverseL = pickMotorDist(-1);
                            linegapStartReverseR = pickMotorDist(1);
                            linegapStartDistL = pickMotorDist(-1);
                            linegapStartDistR = pickMotorDist(1);
                            linegapState ++;
                            endLineGap = false;
                            linegap_millis = millis();
                            // Robawt.resetPID();
                        }
                        break;

                    case 2: //^ scan right 
                        Robawt.setSteer(30, 1);
                        linegapTurnRightDist = fabs(pickMotorDist(1) - linegapStartDistR);
                        // if (millis() - linegap_millis < 200) 
                        // { Robawt.setSteer(0, 0); Robawt.resetPID(); }
                        // else { Robawt.setSteer(30, 1); }
                        if (endLineGap) {
                            linegapStartReverseL = pickMotorDist(-1);
                            linegapStartReverseR = pickMotorDist(1);
                            linegapStartDistL = pickMotorDist(-1);
                            linegapStartDistR = pickMotorDist(1);
                            linegap_rotation = 1;
                            endLineGap = false;
                            linegapState = 7;
                            linegap_millis = millis();
                        }
                        else if (linegapTurnRightDist >= linegapTurnLeftDist){ //^ turns right to match left dist
                            linegapStartReverseL = pickMotorDist(-1);
                            linegapStartReverseR = pickMotorDist(1);
                            linegapStartDistL = pickMotorDist(-1);
                            linegapStartDistR = pickMotorDist(1);
                            endLineGap = false;
                            linegapState ++;
                            linegap_millis = millis();
                        }
                        
                        break;

                    case 3: //^ return to original pos
                        // if (millis() - linegap_millis < 1000)
                        // { Robawt.setSteer(0, 0); Robawt.resetPID(); }
                        // else{ Robawt.setSteer(30, -1);}
                        Robawt.setSteer(30, -1);
                        if (fabs(pickMotorDist(1) - linegapStartReverseR) >= linegapTurnRightDist - 0.15) {
                            if (linegap_rotation == -1) { 
                                linegapStartReverseL = pickMotorDist(-1);
                                linegapStartReverseR = pickMotorDist(1);
                                linegapStartDistL = pickMotorDist(-1);
                                linegapStartDistR = pickMotorDist(1);
                                linegap_millis = millis();
                                linegapState = 6;
                                endLineGap = false;
                            } else {
                                linegap_rotation = 0;
                                linegapState = 5; //skipping debug case 4
                                // linegapSilverDist = pickMotorDist(-1);
                                // linegapSilverMillis = millis();
                                linegapStartReverseL = pickMotorDist(-1);
                                linegapStartReverseR = pickMotorDist(1);
                                linegapStartDistL = pickMotorDist(-1);
                                linegapStartDistR = pickMotorDist(1);
                                linegap_millis = millis();
                                endLineGap = false;
                            }
                        }
                        
                        break;

                    // case 4: //^ actual gap or silver tape?
                        // digitalWrite(LEDPIN, HIGH);
                        // //~ Moving back
                        // // if (fabs(pickMotorDist(-1) - linegapSilverDist) > 4) { //^ move back a few cm to expand FOV
                        // //     Robawt.setSteer(0, 0);
                        // //     endLineGap = false;
                        // //     linegapSilverDist = pickMotorDist(-1);
                        // //     linegapState ++;
                        // //     digitalWrite(LEDPIN, LOW);
                        // // } else {
                        // //     Robawt.setSteer(-30, 0);
                        // // }
                        // //~ Wait 1s
                        // Robawt.setSteer(0, 0);
                        // Robawt.resetPID();
                        // if (millis() - linegap_millis > 200) {
                        //     endLineGap = false;
                        //     linegapStartReverseL = pickMotorDist(-1);
                        //     linegapStartReverseR = pickMotorDist(1);
                        //     linegapStartDistL = pickMotorDist(-1);
                        //     linegapStartDistR = pickMotorDist(1);
                        //     linegapState ++;
                        //     linegap_millis = millis();
                        //     digitalWrite(LEDPIN, LOW);
                        //     endLineGap = false;
                        // }
                        // break;

                    case 5: //^ move forward a bit
                        //~ Moving back forwards
                        // Robawt.setSteer(30, 0);
                        // if (fabs(pickMotorDist(-1) - linegapSilverDist) > 7) { 
                        //     endLineGap = false;
                        //     linegap_millis = millis();
                        //     linegapState ++;
                        // }
                        //~ Wait
                        // linegapStartReverseL = pickMotorDist(-1);
                        // linegapStartReverseR = pickMotorDist(1);
                        // linegapStartDistL = pickMotorDist(-1);
                        // linegapStartDistR = pickMotorDist(1);
                        // linegapState ++ ;
                        // linegap_millis = millis();
                        // endLineGap = false;
                        //~ Move forward
                        Robawt.setSteer(30, 0);
                        if (fabs(pickMotorDist(-1) - linegapStartDistL) >= 5){ 
                            linegapStartReverseL = pickMotorDist(-1);
                            linegapStartReverseR = pickMotorDist(1);
                            linegapStartDistL = pickMotorDist(-1);
                            linegapStartDistR = pickMotorDist(1);
                            linegapState ++;
                            linegap_rotation = 0;
                            endLineGap = false;
                        }
                        break;
                     
                    case 6: //^ turn to the desired direction
                        if (linegap_rotation == -1) { //line on left
                            Robawt.setSteer(30, -1);
                            if (fabs(pickMotorDist(-1) - linegapStartDistL) >= linegapTurnLeftDist || endLineGap){ 
                                linegapState = 7;
                            } else if (millis() - linegap_millis > 2500) { //this is dumb
                                Robawt.setSteer(-30, 0);
                                Serial.println("REVERSING 倒车， 倒车");
                            }
                        } else if (linegap_rotation == 0) { //real line gap
                            Robawt.setSteer(rpm, rotation);
                            if (endLineGap) {
                                linegapState = 7;
                            }
                        }
                        break;

                    case 7: //end line gap, go back lt
                        curr = 0;
                        linegapState = 0;
                        endLineGap = false;
                        linegap_rotation = 0;
                        endLinegapMillis = millis();
                        small_linegapend = false; // paranoia
                        leftSaw = false;
                        rightSaw = false;
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
                        
                    case 8: //reverse a bit less for see saw
                        Robawt.setSteer(-30, 0);
                        if (fabs(pickMotorDist(-1) - linegapStartDistL) > 6) {
                            linegapState = 0;
                            linegapStartDistL = pickMotorDist(-1);
                            linegapStartDistR = pickMotorDist(1);
                            seesaw_now = false;
                        }
                        break;
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

            // case 23: //^ post cube - initialisating vars
            //     send_pi(0);
            //     #if debug_led
            //     led_on = true;
            //     #endif
            //     kitStartReverseDist = pickMotorDist(prev_kit_rotation);
            //     kitDistToReverse = pickMotorDist(prev_kit_rotation) - kitBeforeStraightDist;
            //     curr = 24;
            //     break;

            // case 24: //^ post cube reversing
            //     send_pi(0);
            //     #if debug_led
            //     led_on = true;
            //     #endif
            //     Robawt.setSteer(-40, 0);
            //     if (fabs(pickMotorDist(prev_kit_rotation) - kitStartReverseDist) > kitDistToReverse) { 
            //         kit_distToTurn = kitBeforeStraightDist - kitStartDist;
            //         kitStartTurnBackDist = pickMotorDist(prev_kit_rotation);
            //         curr = 25; }
            //     break;

            // case 25: //^ post cube turning
            //     send_pi(0);
            //     #if debug_led
            //     led_on = true;
            //     #endif
            //     Robawt.setSteer(-rpm, prev_kit_rotation);
            //     kitTurnBackDist =  abs(pickMotorDist(prev_kit_rotation)-kitStartTurnBackDist);
            //     if (kitTurnBackDist > kit_distToTurn) { 
            //         endBlueMillis = millis();
            //         curr = afterKitState; }
            //     break;

            case 26: //^ pickup for rescue kit
                send_pi(0);
                switch (pickupKitState)
                {
                    case 0:
                        pickupKitStateTimer = millis();
                        pickupKitState ++;
                        sort_alive();
                        break;

                    case 1:
                        claw_close_cube();
                        if (millis() - pickupKitStateTimer > 500) {
                            pickupKitStateTimer = millis();
                            pickupKitState ++; }
                        break;

                    case 2:
                        claw_up();
                        if (millis() - pickupKitStateTimer > 500) {
                            pickupKitStateTimer = millis();
                            pickupKitState ++; }
                        break;

                    case 3: 
                        claw_open();
                        if (millis() - pickupKitStateTimer > 500) {
                            pickupKitStateTimer = millis();
                            pickupKitState ++; }
                        break;

                    case 4:
                        claw_down();
                        curr = 0;
                        pickupKitStateTimer = millis();
                        pickupKitState = 0;
                        // if (millis() - pickupKitStateTimer > 1000) {
                        //     pickupKitStateTimer = millis();
                        //     pickupKitState = 0; 
                        //     curr = 23; }
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
                if (see_line && (millis() - obst_time_start) > 2000){
                    curr = 33;
                    obst_time_start = millis();
                    obstState = 0; 
                    obstStartTurnBackDist = pickMotorDist(turn_dir); }
                break;

            // case 33: //^ turning back to line after obstacle (real one)
            //     send_pi(0);
            //     Robawt.setSteer(30, turn_dir*0.7);
            //     obstCurrDist = pickMotorDist(turn_dir);
            //     if (obstCurrDist - obstStartTurnBackDist > 25) { curr = 0; }
            //     break;

            case 33: //^ turning back to line after obstacle (hardcode)
                send_pi(0);
                Robawt.setSteer(30, turn_dir*0.7);
                obstCurrDist = pickMotorDist(turn_dir);
                if (obstCurrDist - obstStartTurnBackDist > 25) { curr = 0; }
                break;


            // case 34: //! hard code for field 2 (remove after)
            //     send_pi(0);
            //     Robawt.setSteer(30, -1);
            //     if (fabs(pickMotorDist(-1) - obstCurrDist) > 19) {
            //         obstCurrDist = pickMotorDist(turn_dir);
            //         if (obstCurrDist - obstStartTurnBackDist > 25) { curr = 0; }
            //     }
            //     break;

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
                        Robawt.setSteer(evac_rpm, 0);
                        if (ballType == 1) { sort_dead(); } //balltype1 should be black right (if its wrong just switch the sort alives and dead)
                        else if (ballType == 0) { sort_alive(); }
                        if (millis() - pickupStateTimer > 500) {
                            pickupStateTimer = millis();
                            pickupState ++; 
                            Robawt.setSteer(0, 0);
                            Robawt.resetPID(); }
                        break;

                    case 2:
                        if (ballType == 1) { sort_dead(); }
                        else if (ballType == 0) { sort_alive(); }
                        Robawt.setSteer(-30, 0);
                        if (millis() - pickupStateTimer > 1000) {
                            pickupStateTimer = millis();
                            pickupState ++; }
                        break;
                        
                    case 3:
                        claw_up();
                        Robawt.setSteer(0, 0);
                        Robawt.resetPID();
                        if (ballType == 1) { sort_dead(); }
                        else if (ballType == 0) { sort_alive(); }
                        if (millis() - pickupStateTimer > 500) {
                            pickupStateTimer = millis();
                            pickupState ++; }
                        break;

                    case 4: 
                        claw_open();
                        Robawt.setSteer(0, 0);
                        Robawt.resetPID();
                        if (ballType == 1) { sort_dead(); }
                        else if (ballType == 0) { sort_alive(); }
                        if (millis() - pickupStateTimer > 500) {
                            pickupStateTimer = millis();
                            pickupState ++; }
                        break;

                    case 5:
                        claw_down();
                        claw_halfclose();
                        sort_neutral();
                        Robawt.setSteer(0, 0);
                        Robawt.resetPID();
                        // if (millis() - pickupStateTimer > 1000) {
                        pickupStateTimer = millis();
                        pickupState = 0; 
                        curr = afterPickupState; 
                        break;
                }
                break;

            case 51: //^ evac initialisation
                send_pi(1);
                lastSerialPiSend = millis();
                switch (enterEvacState) {
                    case 0: // initialise
                        // startEvacMillis = millis();
                        in_evac = true;
                        pickType = 1;
                        depositType = 1;
                        enterEvacState++;
                        evac_startdist = pickMotorDist(-1);
                        startEvacMillis = millis();
                        seeLeftForEvac = false;
                        seeRightForEvac = false;
                        break;

                    case 1: // move forward
                        Robawt.setSteer(40, 0);
                        if (left_see_reallyclosewall()) { seeLeftForEvac = true; }
                        if (right_see_reallyclosewall()) { seeRightForEvac = true; }
                        if (fabs(pickMotorDist(-1) - evac_startdist) > 36) {
                            if (seeLeftForEvac && seeRightForEvac) {
                                evac_startdist = pickMotorDist(-1);
                                enterEvacState++;   
                            } else {
                                enterEvacState = 6;
                                evac_startdist = pickMotorDist(-1);
                            }
                        }
                        break;

                    case 2: // turn left 90
                        Robawt.setSteer(40, -1);
                        if (fabs(pickMotorDist(-1) - evac_startdist > 16.5)) {
                            evac_startdist = pickMotorDist(-1);
                            enterEvacState++;
                        }
                        break;
                        
                    case 3:  //move forward for a short amt of time
                        Robawt.setSteer(evac_rpm, 0);
                        if (l0x_readings[L0X::FRONT] < 120) {  //wall right in front
                            enterEvacState ++;
                            evac_startdist = pickMotorDist(-1);
                        } else if (fabs(pickMotorDist(-1) - evac_startdist) > 42) { //otherwise move this short amount and jump to case 60
                            enterEvacState = 0;
                            startEvacMillis = millis();
                            evac_startdist = pickMotorDist(-1);
                            if (depositedAlready) { curr = 90; }
                            else { curr = 60; }
                        }
                        break;

                    case 4: //wall in front of it, need reverse and turn 90
                        Robawt.setSteer(-evac_rpm, 0);
                        if (fabs(pickMotorDist(-1) - evac_startdist) > 10) {
                            enterEvacState ++;
                            evac_startdist = pickMotorDist(-1);
                        }
                        break;

                    case 5: //turning 90 right from wall in front of it
                        Robawt.setSteer(evac_rpm, 1);
                        if (fabs(pickMotorDist(-1) - evac_startdist) > 18) {
                            if (depositedAlready) { curr = 90; }
                            else { curr = 60; }
                            enterEvacState = 0;
                            startEvacMillis = millis();
                            evac_startdist = pickMotorDist(-1);
                        }
                        break;

                    case 6:
                        Robawt.setSteer(-40, 0);
                        if (fabs(pickMotorDist(-1) - evac_startdist) > 36) {
                            curr = 0;
                            in_evac = false;
                            pickType = 1;
                            depositType = 1;
                            enterEvacState = 0;
                        }
                        break;

                }
                if (ball_present()) { claw_close(); }
                break;

            case 52: //^ rescue kit first forward (not actually used for now)
                Robawt.setSteer(40, 0);
                if ((millis() - startEvacMillis) > 2000) {
                    curr = 71;
                    startEvacMillis = millis();
                    depositType = 2; //rescue kit type
                }
                break;

            case 53: //^ post rescue kit (not actually used for now)
                send_pi(1);
                switch (postRescueKitState)
                {
                    case 0:
                        Robawt.setSteer(-40, 0);
                        if (fabs(pickMotorDist(-1) - postRescueKitTurnDist) > 5) {
                            postRescueKitState ++;
                            postRescueKitTurnDist = pickMotorDist(-1);
                            sort_neutral();
                        }
                        break;

                    case 1:
                        Robawt.setSteer(40, 1);
                        if (fabs(pickMotorDist(-1) - postRescueKitTurnDist) > 35) {
                            curr = 60;

                        }
                        break;
                }
                break;

            case 60: //^ no ball walltrack
                send_pi(1);
                claw_halfclose();
                // if (startEvacMillis - LineTrackStartTime > 300000) { //5 mins spent, less than 3 mins left
                //     evac_time_constant = 135000 / (480000 - (startEvacMillis - LineTrackStartTime) - 45000);
                //     evac_time_constant = 2;
                //     evac_settime = (long int)((millis() - (startEvacMillis + 40000)) * evac_time_constant);
                // } else {
                evac_settime = millis() - (startEvacMillis+40000);
                // }
                if (evac_settime < 0) {
                    evac_settime = 1;
                    #if debug_led
                    led_on = true;
                    #endif
                }
                evac_setdist = 100 + (evac_settime/200);
                if (evac_setdist > 600) {evac_setdist = 600;}
                k_p_wall_rot = 0.008;
                wall_rot = (evac_setdist - l0x_readings[L0X::FRONT_LEFT]) * k_p_wall_rot;
                Robawt.setSteer(evac_rpm, wall_rot);
                Serial.print("evac_settime");
                Serial.println(evac_settime);
                Serial.print("evac_kp");
                Serial.println(k_p_wall_rot);
                if (ball_present()) { 
                    curr = 50;
                    pickupState = 0;
                    afterPickupState = 60; }
                else if (OOR_present()) {
                    startReverseOORDist = MotorL.getDist();
                    curr = 65;
                    OORTurnState = 0; 
                    afterTurnEvacState = 60; 
                }




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
                claw_halfclose();
                #if debug_led
                led_on = true;
                #endif
                // if (rotation < 0.5) { claw_halfclose(); }
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
                        Robawt.setSteer(evac_rpm, 0.15);
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

            case 71: //^ centering for deposit
            //TODO: check if the ball gets stuck when navigating to deposit code works
                if (depositType == 1 || depositType == 2) { send_pi(2); }
                else { send_pi(3); }
                #if debug_led
                led_on = true;
                #endif
                evac_setdist = 600;
                if (frontLeft_see_infinity()) { frontLeftForDepositVal = 750; }
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
                claw_open();
                if (depositType == 1 || depositType == 2) { send_pi(2); }
                else { send_pi(3); }
                // if (fabs(pickMotorDist(-1) - startHeadingToDepositDist) < 1) { Robawt.setSteer(evac_deposit_rpm, 1); }
                // else { Robawt.setSteer(evac_deposit_rpm, rotation); }
                Robawt.setSteer(evac_deposit_rpm, rotation);
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
                    startReverseDistAfterDepL = pickMotorDist(1); 
                } else if (depositType == 2) { //rescue kit
                    curr = 53;
                } else { 
                    send_pi(4);
                    curr = 89;
                    depositToExitState = 0; } // get out of evac
                break;

            case 74: //^ reversing from alive zone
                send_pi(3);
                depositedAlready = true;
                Robawt.setSteer(-evac_deposit_rpm, 0);
                if (fabs(pickMotorDist(1) - startReverseDistAfterDepL) > 60) {
                    curr = 71;
                    depositType = 0;
                }
                break;

            // case 75: //^ after dead zone
            //     send_pi(4);
            //     Robawt.setSteer(-evac_deposit_rpm, 0);
            //     if (fabs(pickMotorDist(1) - startReverseDistAfterDepL) > 60) {
            //         curr = 89;
            //         depositToExitState = 0;
            //     }
            //     break;

            case 80: //^ deposit
                Robawt.setSteer(0, 0);
                Robawt.resetPID();
                switch (depositState)
                {
                    case 0:
                        sort_neutral();
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

            case 89: //^ after dead zone
                send_pi(4);
                switch (depositToExitState) {
                    case 0: //initialise
                        depositToExitDist = pickMotorDist(-1);
                        depositToExitState ++;
                        break;

                    case 1:
                        Robawt.setSteer(-evac_exit_rpm, 0);
                        if (fabs(pickMotorDist(-1) - depositToExitDist) > 10)
                        {
                            depositToExitDist = pickMotorDist(-1);
                            depositToExitState ++;
                        }
                        break;

                    case 2:
                        Robawt.setSteer(evac_exit_rpm, 1);
                        if (fabs(pickMotorDist(-1) - depositToExitDist) > 16.5)
                        {
                            depositToExitState = 0;
                            curr = 90;
                        }
                        break;
                }
                break; 

            case 90: //^ get out of evac by wall tracking and scanning
                send_pi(4);
                depositedAlready = true;
                claw_open();
                #if debug_led
                led_on = true;
                #endif
                evac_setdist = 200;
                wall_rot = (evac_setdist - l0x_readings[L0X::FRONT_LEFT]) * 0.005;
                Robawt.setSteer(evac_exit_rpm, wall_rot);
                // if (frontLeft_see_out() && front_see_out()) {
                //     curr = 91;
                //     evacExitState = 0;
                //     startTurnEvacToLtDist = pickMotorDist(-1);
                if (frontLeft_see_out()) {
                    curr = 93;
                    evacExitState = 0;
                } else if (left_see_out()) {
                    startTurnEvacToLtDist = pickMotorDist(-1);
                    curr = 92;
                    evacExitState = 0;
                }
                if (ball_present()) {
                    curr = 50;
                    pickupState = 0;
                    afterPickupState = 90;
                }
                break;

            case 91: //^ checking if line if facing OOR on left (or) front || DOM: you mean and???
                send_pi(4);
                Serial.print("Evac exit state: ");
                Serial.println(evacExitState);
                switch (evacExitState)
                {
                    case 0:
                        // Robawt.setSteer(-evac_exit_rpm, 0);
                        // if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 5) {
                            evacExitState ++;
                            startTurnEvacToLtDist = pickMotorDist(-1);
                        // }
                        break;

                    case 1:
                        Robawt.setSteer(evac_exit_rpm, 0);
                        if (right_see_wall()) { //this is if frontis infinity
                            evacExitState ++;
                            startTurnEvacToLtDist = pickMotorDist(-1);
                        }
                        else if (left_see_out()) { //when right never trigger, meaning left is infinity
                            evacExitState = 4;
                            startTurnEvacToLtDist = pickMotorDist(-1);
                        }
                        break;

                    case 2: //reverse when front is gap
                        Robawt.setSteer(-evac_exit_rpm, 0);
                        if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 8) {
                            evacExitState ++;
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            startTurnEvacToLtMillis = millis();
                        }
                        break;

                    case 3: //wait for front gap
                        Robawt.setSteer(0, 0);
                        if (millis() - startTurnEvacToLtMillis > 800) {
                            evacExitState = 0;
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            startTurnEvacToLtMillis = millis();
                            curr = 97; // not black
                        }
                        break;

                    case 4: //45 case reverse
                        Robawt.setSteer(-evac_exit_rpm, 0);
                        if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 4) {
                            curr = 92; 
                            evacExitState = 0;
                            startTurnEvacToLtDist = pickMotorDist(-1);
                        }
                        break;
                }
                if (front_see_out() && foundLine) { 
                    startTurnEvacToLtMillis = millis();
                    curr = 96; }
                break;

            case 92: //^ checking if line when left is OOR
                curr = 98;
                break; 

            case 93: //^ when front left see out, wait for left to see out
                send_pi(4);
                Robawt.setSteer(evac_exit_rpm, 0);
                if (left_see_out()) { 
                    startTurnEvacToLtDist = pickMotorDist(-1);
                    evacExitState = 0;
                    curr = 92;
                }
                if (front_see_out() && foundLine) { 
                    startTurnEvacToLtMillis = millis();
                    curr = 96; }
                break;

            case 96: //^ turn to line
                send_pi(0);
                Robawt.setSteer(evac_exit_rpm, 0);
                if (left_see_reallyclosewall()) { leftSawExtensions = true; }
                if (right_see_reallyclosewall()) { rightSawExtensions = true; }
                if ((millis() - startTurnEvacToLtMillis > 1200) || (leftSawExtensions && rightSawExtensions)) { 
                    curr = 0;
                    afterEvac = true;
                    leftSawExtensions = false;
                    rightSawExtensions = false;
                }
                break;

            case 97: //^ turn back to go wall track evac, didnt detect any evac
                Serial.print("Evac exit state: ");
                Serial.println(evacExitState);
                switch (evacExitState)
                {
                    case 0: //reverse
                        Robawt.setSteer(-evac_exit_rpm, 0);
                        if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 8.5) {
                            evacExitState ++;
                            startTurnEvacToLtMillis = millis();
                            startTurnEvacToLtDist = pickMotorDist(-1);
                        }
                        break;

                    case 1: //turn 90
                        Robawt.setSteer(evac_exit_rpm, 1);
                        if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 16.5) {
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            startTurnEvacToLtMillis = millis();
                            evacExitState ++;
                        }
                        break;

                    case 2: //move forward
                        Robawt.setSteer(evac_exit_rpm, 0);
                        if (l0x_readings[L0X::FRONT_LEFT] < 200) { //found wall
                            evacExitState ++;
                            startTurnEvacToLtDist = pickMotorDist(-1);
                        } else if (right_see_closewall()) {
                            curr = 91;
                            evacExitState = 0;
                            startTurnEvacToLtDist = pickMotorDist(-1);
                        } else if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 23) { //otherwise go one tile for two adjacent case
                            curr = 90;
                            evacExitState = 0;
                            startTurnEvacToLtMillis = millis();
                            startTurnEvacToLtDist = pickMotorDist(-1);
                        }
                        break;

                    case 3:  //go past wall, but for shorter amt of time
                        Robawt.setSteer(evac_exit_rpm, 0);
                        if (l0x_readings[L0X::FRONT] < 50) {  //wall right in front
                            evacExitState ++;
                            startTurnEvacToLtDist = pickMotorDist(-1);
                        } else if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 25) { //otherwise move this short amount
                            curr = 90;
                            evacExitState = 0;
                            startTurnEvacToLtMillis = millis();
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            if (left_see_out()) { 
                                startTurnEvacToLtMillis = millis();
                                startTurnEvacToLtDist = pickMotorDist(-1);
                                evacExitState = 6; }
                        }
                        break;

                    case 4: //wall in front of it, need reverse and turn 90
                        Robawt.setSteer(-evac_exit_rpm, 0);
                        if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 10) {
                            evacExitState ++;
                            startTurnEvacToLtDist = pickMotorDist(-1);
                        }
                        break;

                    case 5: //turning 90 from wall in front of it
                        Robawt.setSteer(evac_exit_rpm, 1);
                        if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 18) {
                            curr = 90;
                            evacExitState = 0;
                            startTurnEvacToLtMillis = millis();
                            startTurnEvacToLtDist = pickMotorDist(-1);
                        }
                        break;

                    case 6:
                        Robawt.setSteer(evac_exit_rpm, -0.5);
                        if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 40) {
                            evacExitState = 6;
                            curr = 92; //go to wait
                        }
                        break;
                }
                break;

            case 98: //^ FOR COMPS cuz desperate; checking if line when left is OOR 
                send_pi(4);
                Serial.print("Turned: ");
                Serial.println(turnedEvacExitDist);
                Serial.print("Evac state: ");
                Serial.println(evacExitState);
                Serial.print("Front see infinity: ");
                Serial.println(evacExitFrontVal);
                Serial.print("Move dist: ");
                Serial.println(fabs(pickMotorDist(-1) - startTurnEvacToLtDist));

                switch (evacExitState) {
                        
                    case 0: //move forward
                        Robawt.setSteer(evac_exit_rpm, 0);
                        // if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 3) {
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            evacExitFrontVal = l0x_readings[L0X::FRONT];
                            // if (evacExitFrontVal <= 300) { evacExitState = 15; } //front in front, can be coming in from 45 or real 
                            // else {
                            evacExitState ++; 
                            // isWallInFront = false; 
                            // }
                        // }
                        break;

                    case 1:
                        if (evacExitFrontVal < 400 || evacExitFrontVal > 1500) {
                            evacExitState = 2; //45
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            startTurnEvacToLtMillis = millis();
                        } else {
                            evacExitState = 20; //90
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            startTurnEvacToLtMillis = millis();
                        }
                        break;

                    case 2:
                        Robawt.setSteer(evac_exit_rpm, 0);
                        if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 5) {
                            evacExitState ++;
                            startTurnEvacToLtMillis = millis();
                        }
                        break;

                    case 3: //trigger right sensor for 45
                        Robawt.setSteer(evac_exit_rpm, -0.8);
                        turnedEvacExitDist = fabs(pickMotorDist(-1) - startTurnEvacToLtDist);
                        if (right_see_reallyclosewall()) {
                            evacExitState ++; //first exit
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            startTurnEvacToLtMillis = millis();
                        }
                        break;

                    case 4: // turning back from right see closewall
                        Robawt.setSteer(-evac_exit_rpm, -0.5);
                        if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 3) {
                            if (turnedEvacExitDist < 25) { evacExitState = 6; } //need to reverse
                            else { evacExitState = 7; }
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            startTurnEvacToLtMillis = millis();
                        // }
                        break;

                    case 6: //moving back less if turned less
                        Robawt.setSteer(-evac_exit_rpm, 0);
                        if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 6) {
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            startTurnEvacToLtMillis = millis();
                            evacExitState = 8;
                        }
                        break;

                    case 7: //moving back more if turn more
                        Robawt.setSteer(-evac_exit_rpm, 0);
                        if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 11) {
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            startTurnEvacToLtMillis = millis();
                            evacExitState ++;
                        }
                        break;

                    case 8: //wait
                        Robawt.setSteer(0, 0);
                        if (millis() - startTurnEvacToLtMillis > 800) {
                            evacExitState = 0;
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            startTurnEvacToLtMillis = millis();
                            curr = 97; // not black
                        }
                        break;

                    // case 9: //turn 90 to check if infinity is another evac exit
                    //     Robawt.setSteer(evac_exit_rpm, -0.5);
                    //     // turnedEvacExitDist = fabs(pickMotorDist(-1) - startTurnEvacToLtDist);
                    //     if (right_see_closewall()) { //detects wall after infinity ==> another evac exit
                    //         startTurnEvacToLtMillis = millis();
                    //         startTurnEvacToLtDist = pickMotorDist(-1);
                    //         evacExitState = 4;
                    //     }
                    //     else if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 60){ //turned > 90 in total
                    //         startTurnEvacToLtMillis = millis();
                    //         startTurnEvacToLtDist = pickMotorDist(-1);
                    //         evacExitState = 10;
                    //     }
                    //     break;

                    // case 10:
                    //     Robawt.setSteer(0,0);
                    //     Robawt.resetPID();
                    //     if (millis() - startTurnEvacToLtMillis > 1000){
                    //         evacExitState ++;
                    //     }
                    //     break;

                    // case 11: //turn back 90
                    //     Robawt.setSteer(-evac_exit_rpm, -0.5);
                    //     if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 30) { //so that in total it turns 90
                    //         startTurnEvacToLtMillis = millis();
                    //         startTurnEvacToLtDist = pickMotorDist(-1);
                    //         evacExitState = 5;
                    //         turnedEvacExitDist = 37;
                        }
                        break;

                    case 15: //wait for infinity to trigger after close wall
                        Robawt.setSteer(evac_exit_rpm, -0.5);
                        if (right_see_infinity()) {
                            startTurnEvacToLtMillis = millis();
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            evacExitState = 5;
                        }
                        break;
                    
                    // case 15: //check for front because we need to know if 
                    //     // ------         -------
                    //     // e    |         |     |
                    //     // |    |         |     |
                    //     // |    |    OR   e     |
                    //     // |    |         |     |
                    //     // ------         -------
                    //     Robawt.setSteer(evac_exit_rpm, 1);
                    //     if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 4) {
                    //         if (l0x_readings[L0X::FRONT] > 400) 
                    //         {
                    //             isWallInFront = false;
                    //         } else {
                    //             isWallInFront = true;
                    //         }
                    //         evacExitState = 16;
                    //         startTurnEvacToLtDist = pickMotorDist(-1);
                    //     }
                    //     break;

                    // case 16: //turn back from the 45 degree turn
                    //     Robawt.setSteer(evac_exit_rpm, -1);
                    //     if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 4) {
                    //         // if (isWallInFront) {
                    //         //     evacExitState = 17;
                    //         //     startTurnEvacToLtDist = pickMotorDist(-1);
                    //         //     startTurnEvacToLtMillis = millis();
                    //         // } else {
                    //         //     evacExitState = 4;
                    //         //     startTurnEvacToLtDist = pickMotorDist(-1);
                    //         //     startTurnEvacToLtMillis = millis();
                    //         // }
                    //         if (isWallInFront) { evacExitState = 1; }
                    //         else { evacExitState = 17; }
                    //         startTurnEvacToLtDist = pickMotorDist(-1);
                    //         startTurnEvacToLtMillis = millis();
                    //     }
                    //     break;

                    // case 17: //reverse if 
                    //     // ------
                    //     // |    |
                    //     // |    |
                    //     // e    |
                    //     // |    |
                    //     // -----
                    //     Robawt.setSteer(-evac_exit_rpm, 0);
                    //     if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 10) {
                    //         evacExitState = 1;
                    //         startTurnEvacToLtDist = pickMotorDist(-1);
                    //         startTurnEvacToLtMillis = millis();
                    //     }
                    //     break;

                    case 20: //trigger 90 n move forward
                        Robawt.setSteer(evac_exit_rpm, 0);
                        if (fabs(pickMotorDist(-1) - startTurnEvacToLtDist) > 7) {
                            evacExitState ++;
                            startTurnEvacToLtMillis = millis();
                        }
                        break;

                    case 21:
                        Robawt.setSteer(evac_exit_rpm, -0.6);
                        turnedEvacExitDist = fabs(pickMotorDist(-1) - startTurnEvacToLtDist);
                        if (right_see_reallyclosewall()) {
                            evacExitState = 4; //first exit
                            startTurnEvacToLtDist = pickMotorDist(-1);
                            startTurnEvacToLtMillis = millis();
                        }
                        break;
                
                }
                if (front_see_out() && foundLine) { 
                    startTurnEvacToLtMillis = millis();
                    curr = 96; }
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

        leftSaw = false;
        rightSaw = false;

        linetrackleftSaw = false;

        linetrackrightSaw = false;

        Robawt.setSteer(0, 0);
        Robawt.resetPID();
        
        #if debug_evac
        curr = 51; //! force evac
        depositType = 1;
        #elif debug_evac_exit
        curr = 90;
        foundLine = false;
        evacExitState = 0;
        #elif debug_deposit
        curr = 71;
        depositType = 1;
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
    if (!piOn) { led_on = true; }
    if (led_on) { digitalWrite(ONBOARDLEDPIN, HIGH); }
    else { digitalWrite(ONBOARDLEDPIN, LOW); }
    if (piOn) { led_on = false; }
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
    Serial.print("Motor L Enc: ");
    Serial.print(MotorL.getEncVal());
    Serial.print(" || Motor R Enc: ");
    Serial.print(MotorR.getEncVal());
    Serial.println();
    #endif

    #if debug_evac_speedup
    Serial.print("Start linetrack millis: ");
    Serial.println(LineTrackStartTime);
    Serial.println(LineTrackTimeElapsed);
    if (startEvacMillis - LineTrackStartTime > 300000) { Serial.println("linetrack overtimess"); Serial.print("Time elapsed; "); Serial.println(startEvacMillis - LineTrackStartTime);}
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
        if (serialData == 255 || serialData == 254 || serialData == 253 || serialData == 252 || serialData == 251 || serialData == 250 || serialData == 249 || serialData == 248 || serialData == 246) {
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
                    if (curr == 61) { 
                        ballType = (int)serialData; 
                    }
                    break;
                case 249:
                    silverStrip = (bool)serialData;
                    break;
                case 248:
                    piOn = (bool)serialData;
                    break;
                // case 247:
                //     seesaw_now = (bool)serialData;
                //     break;
                case 246:
                    small_linegapend = (bool)serialData;
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
    servos_angle[Servos::ARM] = 180;
    servos_change = true;
}

void alive_up() {
    servos_angle[Servos::ALIVE] = 60;
    servos_change = true;
}

void alive_down() {
    servos_angle[Servos::ALIVE] = 140;
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


//* EVAC FUNCTIONS

bool ball_present() {
    //+45 for the diff sensors' offset physically
    return ((l0x_readings[L0X::FRONT]+45 - l1x_readings[L1X::FRONT_BOTTOM]) > 30 && l1x_readings[L1X::FRONT_BOTTOM] < 80);
}

bool wall_present() {
    return (l0x_readings[L0X::FRONT]+45 < 90 && l1x_readings[L1X::FRONT_BOTTOM] < 90);
}

bool OOR_present() {
    return ((front_see_infinity() && right_see_wall()) //if front sees out, and right either sees wall or OOR
    || (front_see_infinity() && (left_see_infinity() || (frontLeft_see_infinity() && left_see_out())))); //if left and front sees out
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

//* EVAC WALL TRACKING OUT OF EVAC

bool left_see_out() {
    return (l0x_readings[L0X::LEFT] > 400);
}

bool front_see_out() {
    return (l0x_readings[L0X::FRONT] > 400);
}

bool right_see_out() {
    return (l0x_readings[L0X::FRONT] > 600);
}

bool frontLeft_see_out() {
    return (l0x_readings[L0X::FRONT_LEFT] > 600);
}

bool right_see_wall() {
    return (l0x_readings[L0X::RIGHT] < 500);
}

bool left_see_closewall() {
    return (l0x_readings[L0X::LEFT] < 200);
}

bool right_see_closewall() {
    return (l0x_readings[L0X::RIGHT] < 200);
}

bool evacExitFrontValSeeInfinity() {
    return (evacExitFrontVal > 1680);
}

bool left_see_reallyclosewall() {
    return (l0x_readings[L0X::LEFT] < 150);
}

bool right_see_reallyclosewall() {
    return (l0x_readings[L0X::RIGHT] < 150);
}


//* MISC FUNCTIONS

double pickMotorDist(double prev_rotation) { 
    if (prev_rotation > 0) { return MotorL.getDist(); }
    else { return MotorR.getDist(); }
}

bool obstacle_present() {
    return (l1x_readings[L1X::FRONT_BOTTOM] < 85 && l0x_readings[L0X::FRONT] < 40);
}

bool far_obstacle_present() {
    return (l1x_readings[L1X::FRONT_BOTTOM] < 165 && l0x_readings[L0X::FRONT] < 120);
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
        // double distTravelled = fabs(MotorL.getDist()-startDistanceValL) + fabs(MotorR.getDist()-startDistanceValR);
        // if (distTravelled < 120) {
        //     Robawt.setSteer(-40, 0);
        // } else {
        //     Robawt.setSteer(0, 0);
        //     Robawt.resetPID();
        // }
        double distTravelled = fabs(MotorR.getDist()-startDistanceValR);
        if (distTravelled < 5) {
            Robawt.setSteer(50, -0.5);
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