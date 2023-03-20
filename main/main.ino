// --------------------------------------
// main.cpp
//
// Main script on robot
// --------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include "VL53L1X.h"
#include "VL53L0X.h" //^ Note the L0X library is blocking --> if have time can rewrite the function
#include "Vroom.h"
#include <Servo.h>
#include <elapsedMillis.h>

#define TCAADDR 0x70
#define L0XADDR 0x29

//* OBJECT INITIALISATIONS */
Motor MotorR(13, 12, 19, 18); //M2 swapped
Motor MotorL(10, 11, 1, 0); //M1
Vroom Robawt(&MotorL, &MotorR);
VL53L0X front_tof, l_tof, r_tof, fl_tof; //, r_tof;
VL53L1X fb_tof;
Servo servos[6];

//* SERVOS CONSTANTS SETUP */
double servos_angle[6] = {0, 0, 0, 0, 0, 0}; //basic states initialised
const double servos_max_angle[6] = {180, 300, 300, 300, 300, 300};
const int servos_pin[6] = {27, 26, 22, 21, 20, 2};
bool servos_change = false;
namespace Servos {
  enum Servos { ARM, LEFT, RIGHT, S4, S5, S6 };
}
// unsigned long servo_time_start;

//* MOTOR ENCODERS */
void ISRLA() {MotorL.readEncA();}
void ISRLB() {MotorL.readEncB();}
void ISRRA() {MotorR.readEncA();}
void ISRRB() {MotorR.readEncB();}

//* CONSTANTS
const int TX1PIN = 8,
  RX1PIN = 9,
  TX0PIN = 16,
  RX0PIN = 17,
  SDAPIN = 4,
  SCLPIN = 5,
  SWTPIN = 28,
  LEDPIN = 3;

int serialState = 0;
int front_dist = 0,
  fb_dist = 0,
  l_dist = 0,
  r_dist = 0,
  fl_dist = 0;

double rotation = 0;
double rpm = 40;
int task = 39, 
  prev_task = 0, 
  curr = 0;

long lostGSMillis, 
  startGSMillis;

long startEvacMillis;
bool see_ball = false;
bool in_evac = true;

double evac_setdist,
  wall_rot;

//* LOGIC SETUP

double prev_L_dist = 0;
double prev_R_dist = 0;
double start_L_dist = 0;
double start_R_dist = 0;
double rev_L_dist = 0;
double rev_R_dist = 0;
double prev_rotation = 0;

long startBlueMillis = 0;
long timeElapsed = 0;
long startReverse = 0;

elapsedMillis sinceStop;

void setup() {

  pinMode(SWTPIN, INPUT_PULLDOWN);
  pinMode(LEDPIN, OUTPUT);

  //* SERVOS */
  for (int i = Servos::ARM; i != (Servos::S6 + 1); i++) {
    servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
    servos[i].attach(servos_pin[i], 500, 2500); // (pin, min, max)
  }

  //* USB SERIAL COMMS */
  Serial.begin(9600);
  while (!Serial) delay(10);
  Serial.println("USB serial initialised");

  //* PI SERIAL COMMS */
  Serial1.setRX(RX0PIN);
  Serial1.setTX(TX0PIN);
  Serial1.begin(9600); //consider increasing baud
  while (!Serial1) delay(10); 
  Serial.println("Pi serial initialised");

  //* TEENSY SERIAL COMMS */
  // Serial2.setRX(RX1PIN);
  // Serial2.setRX(TX1PIN);
  // Serial2.begin(9600);
  // while (!Serial2) delay(10); 
  // Serial.println("Teensy serial initialised");

  //* MULTIPLEXER */
  businit(&Wire, SDAPIN, SCLPIN);

  // l0xinit(&Wire, &front_tof, 4);
  l0xinit(&Wire, &fl_tof, 3);
  l1xinit(&fb_tof, 2);
  // l0xinit(&Wire, &l_tof, 1);
  //l0xinit(r_tof, 5);

  //* MOTOR ENCODERS */    
  //^ basically interrupts the main code to run the encoder code 
  //^ uhm not just the main code its every code thats what attach interrupt means
  attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
  attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
  attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
  attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

//* -----------END SETUP-----------*//


void loop() {
  
  digitalWrite(LEDPIN, HIGH);

  if (servos_change) {
    for (int i = Servos::ARM; i != (Servos::S6 + 1); i++) {
      servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
    }
    servos_change = false; //^ DOM: not sure if this is correct //^ XEL: why not??
  }

  // tcaselect(1);
  // l_dist = l_tof.readRangeContinuousMillimeters();
  tcaselect(2);
  fb_dist = fb_tof.read();
  tcaselect(3);
  fl_dist = fl_tof.readRangeContinuousMillimeters();
  // tcaselect(4);
  // front_dist = front_tof.readRangeContinuousMillimeters();
  // Serial.print("left: ");
  // Serial.println(l_dist);
  Serial.print("front below: ");
  Serial.println(fb_dist);
  Serial.print("front left: ");
  Serial.println(fl_dist);
  // Serial.print("front: ");
  // Serial.println(front_dist);

  if (curr != 39) serialEvent();

  if (!digitalRead(SWTPIN)) {

    // claw_open();
    claw_up();
    claw_close();

    // test_all_servos();

    //* HANDLING THE INFO RECEIVED */
    switch (task)
    {

      //~ Handling the continue turning after green squares 
      case 0:
        if (curr == 1) {
          curr = 10;
          lostGSMillis = millis();
        } else if (curr == 2) {
          curr = 11;
          lostGSMillis = millis();
        } else if (curr != 10 && curr != 11 && curr != 7) {       //^ DOM: I'll add a temp case for rescue kit :thumbs: 
          curr = 0;
        } 
        break;

      case 1:
        if (curr == 0) {
          startGSMillis = millis();
        }
        curr = 1;
        break;

      case 2:
        if (curr == 0) {
          startGSMillis = millis();
        }
        curr = 1;
        break;

      case 3:
        if (curr == 0) {
          startGSMillis = millis();
        }
        curr = 1;
        break;

      //~ Evac
      case 9: //evac no ball
        see_ball = false;
        if (curr == 41 || curr == 42) curr = 42; //enter post-ball
        else curr = 40;
        break;

      case 10: //evac got ball
        see_ball = true;
        curr = 41;
        break;

      default:
        if ((curr == 1 || curr == 2) && (millis() - startGSMillis > 400)) {
          curr = task;
        } else if (curr == 3 && (millis() - startGSMillis > 600)) {
          curr = task;
        } else if (curr != 40 && curr != 39) {
          curr = task;
        }
        break;

    }

//     //* ACTUAL CODE
    switch (curr) 
    {

      case 0: //normal lt
        Robawt.setSteer(rpm, rotation);
        break;

      case 1: //left gs
        Robawt.setSteer(rpm, -0.5);
        break;
      
      case 2: //right gs
        Robawt.setSteer(rpm, 0.5);
        break;

      case 3: //double gs
        Robawt.setSteer(50, 1);
        break;

      case 4: //red line
        Robawt.setSteer(0, 0);
        Robawt.reset();
        break;

      case 5: //turning towards blue
        Robawt.setSteer(rpm, rotation);
        if (prev_rotation == 0) prev_rotation = rotation;
        if (start_L_dist == 0) {start_L_dist = MotorL.getDist(); start_R_dist = MotorR.getDist();}
        prev_L_dist = MotorL.getDist();
        prev_R_dist = MotorR.getDist();
        //* Debug motor enc values */
//        Serial.print("Left Motor: ");
//        Serial.println(MotorL.getDist());
//        Serial.print("Right Motor: ");
//        Serial.println(MotorR.getDist());
        break;

      case 6: //relatively centred on blue
        Robawt.setSteer(rpm , 0);
        if (startBlueMillis == 0) {startBlueMillis = millis();}
        //TODO: insert code for picking up the block? not sure how that'll work
        /*pseudocode:
        if (front_dist < 30) {
            stop for 1s? 
            pick up 
            if (servos_change) curr = 7;
        }
        */
       break;
       
      case 7: //Reversing bot (reminder to make it pico side only)
        if (timeElapsed == 0) {
          timeElapsed = millis() - startBlueMillis; 
          startReverse = millis(); 
          sinceStop = 0; //temporary; used to stop the bot
        }
        
        if (sinceStop < 2000) Robawt.setSteer(0, 0); //^ DOM: Remove this debug shit later
        else { 
        //& Debug varibles
//        Serial.print("ABS value");
//        Serial.println(abs(MotorL.getDist() - rev_L_dist));
//        Serial.print("prev - start");
//        Serial.println(prev_L_dist - start_L_dist);

        //~ Move backwards first
        if (millis() - startReverse <= timeElapsed) {
          Robawt.setSteer(-rpm, 0);
        }
        
        //~ Rotating in the opposite direction
        else { 
          if (rev_L_dist == 0) {rev_L_dist = MotorL.getDist(); rev_R_dist = MotorR.getDist(); }
          double L_dist_to_travel = prev_L_dist - start_L_dist;
          double R_dist_to_travel = prev_R_dist - start_R_dist;
          double dist_L_Travelled = abs(MotorL.getDist() - rev_L_dist); //^ ABSing cuz distance will decrease when moving backwards
          double dist_R_Travelled = abs(MotorR.getDist() - rev_R_dist);

          //^ If motor hasn't travelled the requisite distance
          if ((dist_L_Travelled <= L_dist_to_travel) && (dist_R_Travelled <= R_dist_to_travel)) {
            Robawt.setSteer(-rpm, prev_rotation);
          }
          else {
            //curr = 0;
            Robawt.setSteer(0, 0);
            Robawt.reset();
//            Serial.print("Finished reversing");
            //^ DOM: reset variables to test blue lolz
            //timeElapsed = 0, startBlueMillis = 0, prev_rotation = 0, start_L_dist = 0, rev_L_dist = 0;
          }
        }
        }
        break;

      case 10: //only pico side --> just lost left green, return to lt
        if (millis() - lostGSMillis > 200) curr = 0;
        else Robawt.setSteer(rpm, -0.5);
        break;

      case 11: //only pico side --> just lost right green, return to lt
        if (millis() - lostGSMillis > 200) curr = 0;
        else Robawt.setSteer(rpm, 0.5);
        break;

      case 39: //initialise evac
        startEvacMillis = millis();
        task = 40;
        curr = 40;
        break;

      case 40: // evac wall track with diagonal lidar
        // claw_open();
        evac_setdist = 140 + (millis() - startEvacMillis)/500;
        if (evac_setdist > 600) {evac_setdist = 600;}
        wall_rot = (evac_setdist - fl_dist) * 0.0095;
        Robawt.setSteer(30, wall_rot);
        break;

      case 41: //ball seen
        claw_halfclose();
        if (fb_dist < 100) {
          // claw_close();
          Robawt.setSteer(0, 0); //! remove after testing
        } else {
          Robawt.setSteer(25, rotation);
        }
        break;

      case 42: //post-ball
        claw_halfclose();
        if (fb_dist < 100) {
          // claw_close();
          Robawt.setSteer(0, 0); //! remove after testing
        } else {
          curr = 40; //go back to wall track
        }
        break;

    }

//     Serial.print("Case");
//     Serial.println(curr);

    //* DEBUG PASSED VARIABLES */
//    Serial.print("task: ");
//    Serial.println(task);
//    Serial.print("rotation: ");
//    Serial.println(rotation);
//    Serial.print("rpm: ");
//    Serial.println(rpm);

    //* TEST PID */
//     double val = MotorL.setRpm(40);
//     Serial.print("Actual rpm: ");
//     Serial.println(MotorL.getRpm());
//     Serial.print(" Output pwm: ");
//     Serial.println(val);
//     Robawt.setSteer(40, 0);   

  } else {
    
    // startChangeMillis = millis();
    // claw_close();
    // Serial.println("0");

    //* TO MAKE ROBOT STOP */
    Robawt.setSteer(0, 0);
    Robawt.reset();
    claw_open();
    claw_down();
    // test_all_servos2();

    //* TO MAKE LEFT MOTOR STOP */
    // MotorL.setRpm(0);
    // MotorL.resetPID();
  }

  //* DEBUG SWITCH STATE (1 is off, 0 is on)*/
  // Serial.print("Switch state");
  // Serial.println(digitalRead(SWTPIN));

    //* DEBUG MOTOR DISTANCES */
//  Serial.print("Distance (cm): ");
//  Serial.println(MotorL.getDist());
//  Serial.print("Angle (deg): ");
//  Serial.println(MotorL.getAngle());
//  Serial.print("Enc Val: ");
//  Serial.println(MotorL.getEncVal());

    //* READ LIDAR */
//    tcaselect(4);
//    front_dist = front_tof.readRangeContinuousMillimeters();
//    Serial.println(front_dist);
}

//* -----------END LOOP-----------*//




//* FUNCTIONS */

void serialEvent()
{
  while (Serial1.available()) {
    int serialData = Serial1.read();
    if (serialData == 255 || serialData == 254 || serialData == 253) {serialState = (int)serialData;}
    else {
      switch (serialState) {
        case 255:
          rotation = ((double)(serialData)-90)/90;
          break;
        case 254:
          rpm = (double)serialData;
          break;
        case 253:
          task = (int)serialData;
          break;
      }
    }
  }
}

void tcaselect(uint8_t i)  //^ DOM: this is the MUX/multiplexer
{
  if (i > 0 && i < 7) {
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
  } else if (i == 8) {
    Wire.beginTransmission(L0XADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
  } else if (i == 9) {
    Wire1.beginTransmission(L0XADDR);
    Wire1.write(1 << i);
    Wire1.endTransmission();
  }
}

void businit(TwoWire *bus, int sdaPin, int sclPin)
{
  bus->setSDA(sdaPin);
  bus->setSCL(sclPin);
  bus->begin();
  bus->setClock(400000); 
}

void l0xinit(TwoWire *bus, VL53L0X *sensor, uint8_t i)
{
  tcaselect(i);
  sensor->setTimeout(500);
  while (!sensor->init()) {Serial.println("L0X failed to initialise");}
  sensor->startContinuous(33);
  sensor->setBus(bus);
}

void l1xinit(VL53L1X *sensor, uint8_t i)
{
  tcaselect(i);
  sensor->setTimeout(500);
  while (!sensor->init()) {Serial.println("L1X failed to initialise");}
  sensor->setDistanceMode(VL53L1X::Medium);
  sensor->startContinuous(50);
}

// int l1xread(VL53L1X *sensor, uint8_t pin)
// {
//   tcaselect(pin);
//   int value = sensor.read();
//   if (sensor.timeoutOccurred()) return -1;
//   else return value;
// }

int pwmangle(double angle, double max_angle) {return (int)(angle/max_angle * 2000 + 500);}

void claw_open() {
  servos_angle[Servos::RIGHT] = 0;
  servos_angle[Servos::LEFT] = 110;
  servos_change = true;
}

void claw_close() {
  servos_angle[Servos::RIGHT] = 110;
  servos_angle[Servos::LEFT] = 0;
  servos_change = true;
}

void claw_up() {
  servos_angle[Servos::ARM] = 0;
  servos_change = true;
}

void claw_down() {
  servos_angle[Servos::ARM] = 180;
  servos_change = true;
}

void claw_halfclose() {
  servos_angle[Servos::RIGHT] = 30;
  servos_angle[Servos::LEFT] = 80;
  servos_change = true;
}

void test_all_servos() {
  servos_angle[Servos::ARM] = 0;
  servos_angle[Servos::RIGHT] = 0;
  servos_angle[Servos::LEFT] = 0;
  // servos_angle[Servos::S4] = 0;
  // servos_angle[Servos::S5] = 0;
  // servos_angle[Servos::S6] = 0;
  servos_change = true;
}

void test_all_servos2() {
  servos_angle[Servos::ARM] = 180;
  servos_angle[Servos::RIGHT] = 300;
  servos_angle[Servos::LEFT] = 300;
  // servos_angle[Servos::S4] = 300;
  // servos_angle[Servos::S5] = 300;
  // servos_angle[Servos::S6] = 300;
  servos_change = true;
}

void send_pi(int i) {
  Serial1.println(i);
}