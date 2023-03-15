// --------------------------------------
// main.cpp
//
// Main script on robot
// --------------------------------------

#include <Arduino.h>
#include <Wire.h>
//#include "VL53L1X.h"
#include "VL53L0X.h" //? Note the L0X library is blocking --> if have time can rewrite the function
#include "Vroom.h"
#include <Servo.h>

#define TCAADDR 0x70
#define L0XADDR 0x29

//* OBJECT INITIALISATIONS */
Motor MotorR(13, 12, 19, 18); //M2 swapped
Motor MotorL(10, 11, 1, 0); //M1
Vroom Robawt(&MotorL, &MotorR);
VL53L0X front_tof; //, r_tof;
Servo servos[2];

//* SERVOS CONSTANTS SETUP */
double servos_angle[2] = {0, 0}; //basic states initialised
const double servos_max_angle[2] = {300, 300};
const int servos_pin[2] = {27, 26};
bool servos_change = false;
namespace Servos {
  enum Servos { LEFT, RIGHT };
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
int front_dist = 0;

double rotation = 0;
double rpm = 40;
int task = 0, 
  prev_task = 0, 
  curr = 0;

long lostGSMillis;


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
  for (int i = Servos::LEFT; i != (Servos::RIGHT + 1); i++) {
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

  //* MULTIPLEXER */
  businit(&Wire, SDAPIN, SCLPIN);

  l0xinit(&Wire, &front_tof, 4);
  //l0xinit(r_tof, 5);

  //* MOTOR ENCODERS */    //^ basically interrupts the main code to run the encoder code
  attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
  attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
  attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
  attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

//* -----------END SETUP-----------*//


void loop() {
  
  digitalWrite(LEDPIN, HIGH); // Turns on LED by setting it to high voltage?

  if (servos_change) {
    for (int i = Servos::LEFT; i != (Servos::RIGHT + 1); i++) {
      servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
    }
    servos_change = false; //^ DOM: not sure if this is correct
  }

  serialEvent();  //^ DOM: Reminder for anyone reading this to uncomment this when testing LOL

  if (!digitalRead(SWTPIN)) {

    //claw_open();

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

      default:
        curr = task;

    }

    //* ACTUAL CODE
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
        //* Debug varibles */
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
        if (millis() - lostGSMillis > 100) curr = 0;
        else Robawt.setSteer(rpm, -0.5);

      case 11: //only pico side --> just lost right green, return to lt
        if (millis() - lostGSMillis > 100) curr = 0;
        else Robawt.setSteer(rpm, 0.5);

    }

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

  } 
  else {
    
    // startChangeMillis = millis();
    // claw_close();
    // Serial.println("0");

    //* TO MAKE ROBOT STOP */
    Robawt.setSteer(0, 0);
    Robawt.reset();

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

// int l0xread(VL53L0X *sensor, uint8_t i)
// {
//   tcaselect(i);
//   return sensor->readRangeContinuousMillimeters();
// }

// void l1xinit(VL53L1X *sensor, uint8_t pin)
// {
//   tcaselect(pin);
//   sensor.setTimeout(500);
//   while (!sensor.init()) {Serial.println("L1X failed to initialise");}
//   sensor.setDistanceMode(VL53L1X::Medium);
//   sensor.startContinuous(50);
// }

// int l1xread(VL53L1X *sensor, uint8_t pin)
// {
//   tcaselect(pin);
//   int value = sensor.read();
//   if (sensor.timeoutOccurred()) return -1;
//   else return value;
// }

int pwmangle(double angle, int max_angle) {return (int)(angle/max_angle * 2000 + 500);}

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
