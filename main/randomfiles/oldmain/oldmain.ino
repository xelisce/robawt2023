  // --------------------------------------
// main.cpp
//
// Main script on pico
// --------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include "VL53L1X.h"
#include "VL53L0X.h" //^ Note the L0X library is blocking --> if have time can rewrite the function
#include "Vroom.h"
#include <Servo.h>
// #include <elapsedMillis.h>

#define TCAADDR 0x70
#define L0XADDR 0x29

//* DEBUG VARIABLES */
//~ Main code runs OR
#define debug_servos 0
#define debug_lidars 0
#define debug_switch 0
#define debug_motors 0
#define debug_pid 0
#define debug_ball 0
#define debug_cube 0
#define debug_teensy_comms 0
//~ Runs WITH main code
#define debug_curr 0
#define debug_passed_vars 1
#define debug_lidars_while 1
#define debug_teensy_vars 1
#define debug_with_led 1

//* OBJECT INITIALISATIONS */
Motor MotorR(13, 12, 19, 18); //M2 swapped
Motor MotorL(10, 11, 1, 0); //M1
Vroom Robawt(&MotorL, &MotorR);
VL53L0X front_tof, l_tof, r_tof, fl_tof;
VL53L1X fb_tof;
Servo servos[6];

//* SERVOS SETUP */
double servos_angle[6] = {180, 0, 135, 0, 0, 0}; //basic states initialised
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

//* CONSTANTS */

double valL, valR;

//~ Pins
const int TNYPIN1 = 9,
  TNYPIN2 = 8,
  TX0PIN = 16,
  RX0PIN = 17,
  SDAPIN = 4,
  SCLPIN = 5,
  SWTPIN = 28,
  LEDPIN = 3,
  ONBOARDLEDPIN = 25;

//~ Multiplexer 
const int F_LIDAR = 4,
  FL_LIDAR = 3,
  FB_LIDAR = 2,
  L_LIDAR = 1,
  R_LIDAR = 7;

//* LOGIC SET UP */

//~ Lidars
int front_dist = 0,
  fb_dist = 0,
  l_dist = 0,
  r_dist = 0,
  fl_dist = 0;

//~ Movement and logic
double rotation = 0,
  rpm = 40;
int serialState = 0,
  task = 39, 
  prev_task = 0,
  curr = 0;

//~ Green squares
long lostGSMillis, 
  startGSMillis;

//~ Evac
long startEvacMillis,
  evacBallTimer;
bool in_evac = false,
  see_ball = false;
double evac_setdist,
  wall_rot;

//~ Blue cube
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
long pickup_timer = 0;
long drop_timer = 0;

//~ Obstacle
bool see_line = false;
int side_dist = 0;
double o_rotation = 0;
int obs_state = 0;
int turn_dir = 1;
double obst_init_dist = 0; //check if this causes any errors
bool obst_reversing = false;
double obst_rotation = 0;
int turning_state = 0;
int obs_time_start = 0;

//~ 135 OR 90
bool left135 = false,
  right135 = false;
long last135Millis, start135Millis, 
  start90Millis;

long loopTimeMillis;

void setup() {

  pinMode(SWTPIN, INPUT_PULLDOWN);
  pinMode(ONBOARDLEDPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(TNYPIN1, INPUT);
  pinMode(TNYPIN2, INPUT);

  last135Millis = millis();

  //* SERVOS */
  for (int i = Servos::ARM; i != (Servos::S6 + 1); i++) {
    servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
    servos[i].attach(servos_pin[i], 500, 2500); // (pin, min, max)
  }

  //* USB SERIAL COMMS */
  Serial.begin(9600);
  // while (!Serial) delay(10);
  Serial.println("USB serial initialised");

  //* PI SERIAL COMMS */
  Serial1.setRX(RX0PIN);
  Serial1.setTX(TX0PIN);
  Serial1.begin(9600); //consider increasing baud
  while (!Serial1) delay(10); 
  Serial.println("Pi serial initialised");

  //* MULTIPLEXER */
  businit(&Wire, SDAPIN, SCLPIN);

  l0xinit(&Wire, &fl_tof, FL_LIDAR);
  l0xinit(&Wire, &front_tof, F_LIDAR);
  
  l1xinit(&fb_tof, FB_LIDAR);
  l0xinit(&Wire, &l_tof, L_LIDAR);
//  l0xinit(&Wire, &r_tof, R_LIDAR);

  //* MOTOR ENCODERS */    
  //^ basically interrupts *every* code to run the encoder code 
  attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
  attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
  attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
  attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

//* -----------END SETUP-----------*//




#if !debug_servos && !debug_lidars && !debug_switch && !debug_motors && !debug_pid && !debug_ball && !debug_cube && !debug_teensy_serial
void loop() 
{

  Serial.print("Time loop: ");
  Serial.println(millis() - loopTimeMillis);
  loopTimeMillis = millis();
  digitalWrite(LEDPIN, HIGH);

  if (servos_change) {
    for (int i = Servos::ARM; i != (Servos::S6 + 1); i++) {
      servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
    }
    servos_change = false;
  }

  //* LIDAR READINGS */

  // tcaselect(L_LIDAR);
  // l_dist = l_tof.readRangeContinuousMillimeters();
  // tcaselect(FB_LIDAR);
  // fb_dist = fb_tof.read();
  // tcaselect(FL_LIDAR);
  // fl_dist = fl_tof.readRangeContinuousMillimeters();
  // tcaselect(F_LIDAR);
  // front_dist = front_tof.readRangeContinuousMillimeters() + 45;
  // tcaselect(R_LIDAR);
  // r_dist = r_tof.readRangeContinuousMillimeters();



  if (!digitalRead(SWTPIN)) {

      #if debug_lidars_while
      Serial.print("left: ");
      Serial.println(l_dist);
      Serial.print("front below: ");
      Serial.println(fb_dist);
      Serial.print("front left: ");
      Serial.println(fl_dist);
      Serial.print("front: ");
      Serial.println(front_dist);
      Serial.print("right: ");
      Serial.println(r_dist);
      #endif

    //* HANDLING THE INFO RECEIVED */

    serialEvent();
    // teensyEvent();

    if (curr != 39) { //case 39 is transitioning into evac
    switch (task)
    {

      //~ Handling the continue turning after green squares 
      case 0:
        if (in_evac) break;

        if (curr == 1) {
          if (millis() - startGSMillis > 500) {
            curr = 10;
            lostGSMillis = millis();
          }
        } else if (curr == 2){
          if (millis() - startGSMillis > 500) {
            curr = 11;
            lostGSMillis = millis();
          }
        } else if (curr == 3){
          if (millis() - startGSMillis > 1000) { //! hard code time
            curr = 0;
          }
        } else if (curr == 25 || curr == 26) {
          if (millis() - start90Millis > 1000) curr = 0;
        } else if (curr != 10 && curr != 11 && curr != 7 && curr != 8) { 
          // if (left135) {
          //   if (curr != 27 && millis() - last135Millis > 1500) {
          //     start135Millis = millis();
          //     curr = 27;
          //   }
          // } else if (right135) {
          //   if (curr != 28 && millis() - last135Millis > 1500) {
          //     start135Millis = millis();
          //     curr = 28;
          //   }
          // } else {
          curr = 0;
          Serial.println("Default pass");
          // }
        }
        break;

      case 1: //left green
        if (in_evac) break;
        if (curr == 3) break;
        if (curr == 0) startGSMillis = millis();
        curr = 1;
        break;

      case 2: //right green
        if (in_evac) break;
        if (curr == 3) break;
        if (curr == 0) startGSMillis = millis();
        curr = 2;
        break;

      case 3: //double green
        if (in_evac) break;
        if (curr == 0) startGSMillis = millis();
        curr = 3;
        break;

      case 5: //reversing from rescue; pi sometimes sends over case 5 even tho its not supposed to 
        if (curr != 7 && curr != 8) curr = 5;
        else Serial.println("it was 5?");
        break;
        
      case 6: //reversing from rescue
        if (curr != 7 && curr != 8) curr = 6;
        else Serial.println("it was 6?");
        break;

      // case 7: //no line
      //   if (curr != 0) break;
      //   if (left135) {
      //       // if (curr != 27 && millis() - last135Millis > 1500) {
      //         start135Millis = millis();
      //         curr = 27;
      //       // }
      //   } else if (right135) {
      //       // if (curr != 28 && millis() - last135Millis > 1500) {
      //         start135Millis = millis();
      //         curr = 28;
      //       // }
      //   } else {
      //     curr = 0;
      //   }
      //   break;

      case 10: //turn left
        curr = 25;
        if (curr != 25) start90Millis = millis();
        break;

      case 11: //turn right
        curr = 26;
        if (curr != 26) start90Millis = millis();
        break;

      //~ Evac
      case 15: //evac no ball
        if (!in_evac) break;
        see_ball = false;
        curr = 40;
        // if (curr == 41) {
        //   curr = 42; //enter post-ball
        //   evacBallTimer = millis();
        // } else if (curr == 42 && (millis() - evacBallTimer) > 1500 && !ball_present()) {
        //   curr = 40; //go wall track
        // } else if (curr == 42) {
        //   curr = 42;
        // }
        break;

      case 16: //evac got ball
        if (!in_evac) break;
        see_ball = true;
        if (curr != 45) curr = 41;
        break;

      default:
        if ((curr == 1 || curr == 2) && (millis() - startGSMillis > 700)) { //min single square turn time
          curr = task; 
        } else if (curr == 3 && (millis() - startGSMillis > 1400)) { //min double green turn time
          curr = task;
        } else if (!in_evac && curr != 8) {
          curr = task;
        }
        break;

    }}
    
    //* ACTUAL CODE CURRENT */

    switch (curr) 
    {

      case 0: //normal lt
        Robawt.setSteer(rpm, rotation);
        // if (fb_dist < 100){
        //   curr = 12;
        //   if (l_dist > r_dist){
        //     turn_dir = -1;
        //   }
        //   else {
        //     turn_dir = 1;
        //   }
        // }
        break;

      case 1: //left gs
        Robawt.setSteer(rpm, -0.6);
        break;
      
      case 2: //right gs
        Robawt.setSteer(rpm, 0.6);
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
        break;

      case 6: //relatively centred on blue
        Robawt.setSteer(rpm , 0);
        claw_down();
        claw_halfclose();
        if (startBlueMillis == 0) {startBlueMillis = millis();}
        if (ball_present()) {
          pickup_timer = millis();
          Robawt.setSteer(0, 0);

          claw_down();
          claw_close_cube();

          timeElapsed = millis() - startBlueMillis;
          curr = 7;       
        }
       break;
       
      case 7: //Reversing bot pt 1
        if (millis() - pickup_timer > 1000) { 
          claw_up();   
          drop_timer = millis();
          curr = 8; 
        }
        else {
          Robawt.setSteer(0, 0);
        }  
        break;

      case 8: //reversing bot pt 2
        if (millis() - drop_timer > 1000){
          claw_open();
          #if debug_with_led
          digitalWrite(ONBOARDLEDPIN, HIGH);
          #endif
          //~ Move backwards first
          if (millis()- startReverse <= timeElapsed) {
            Robawt.setSteer(-rpm, 0);
          }
          
          //~ Rotate in the opposite direction
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
              Robawt.setSteer(0, 0);
              Robawt.reset();
              claw_down();
              #if debug_with_led
              digitalWrite(ONBOARDLEDPIN, LOW);
              #endif
              Serial.println("Finished Reversing");
              curr = 0;
            }
          }
        }
        else {
            Robawt.setSteer(0, 0);
            startReverse = millis();
            Serial.println(startReverse);
        }


      case 10: //only pico side --> just lost left green, return to lt
        if (millis() - lostGSMillis > 400) curr = 0;
        else Robawt.setSteer(rpm, -0.5);
        break;

      case 11: //only pico side --> just lost right green, return to lt
        if (millis() - lostGSMillis > 400) curr = 0;
        else Robawt.setSteer(rpm, 0.5);
        break;
      
      //* OBSTACLE

      case 12: //Obstacle
        if (!obst_reversing){
          obst_init_dist = MotorL.getDist();
          obst_reversing = true;
        }
        Robawt.setSteer(-40, 0);
        if (MotorL.getDist() - obst_init_dist < -12) //TUNE AGAIN, reverse for 3 cm??
        {
          Robawt.setSteer(0, 0);
          Robawt.reset();
          if (turn_dir == 1)// turn right (need to use left lidar)
          {
           side_dist = l_dist;
           o_rotation = -1;
           curr = 14;
           obst_init_dist = MotorL.getDist();
          }
          else if (turn_dir == -1) { // turn left 
            side_dist = r_dist;
            curr = 16;
            o_rotation = 1;
            obst_init_dist = MotorL.getDist();
          }
        }

      case 14: // turn right, use left tof
        switch (turning_state)
        {
          case 0:
          {
            Robawt.setSteer(rpm, 1);
            if (l_dist>150) turning_state ++;
          }
          break;

          case 1:
          {
            Robawt.setSteer(rpm, 1);
            if (l_dist<150) turning_state ++;
          }
          break;

          case 2:
          {
            Robawt.setSteer(rpm, 0);
            turning_state = 0;
            curr = 18;
            obs_time_start = millis();
          }
          break;
        }
        break;

      case 16: // trun left, use right tof
        switch (turning_state)
        {
          case 0:
            Robawt.setSteer(rpm, -1);
            if (r_dist>150) turning_state ++;
            break;

          case 1:
            Robawt.setSteer(rpm, -1);
            if (r_dist<150) turning_state ++;
            break;

          case 2:
            Robawt.setSteer(rpm, 0);
            turning_state = 0;
            curr = 18;
            obs_time_start = millis();
            break;
        }
        break;

      case 18:
        if (turn_dir == 1)// turn right (need to use left lidar)
        {
          side_dist = l_dist;
          o_rotation = -0.5;
        }
        else if (turn_dir == -1){
           side_dist = r_dist;
          o_rotation = 0.5;
        }

        switch (obs_state)
        {
          case 0:
            Robawt.setSteer (25, 0);
            if (side_dist < 200) obs_state++;
            break;
            
          case 1:
            Robawt.setSteer (25, 0);
            if (side_dist > 200) obs_state++;
            break;

          case 2:
            Robawt.setSteer (25, o_rotation);
            if (side_dist < 200) obs_state++;
            break;

           case 3:
            Robawt.setSteer (25, o_rotation);
            if (side_dist > 200){
              obs_state = 0;
            }
            break;
        }

        if (see_line && (millis() - obs_time_start) > 6000){
          Robawt.setSteer(0, 0);
          curr = 19;
          obs_time_start = millis();
          obs_state = 0;
          see_line = false;
        }
        break;
        
      case 19:
        Robawt.setSteer(30, (turn_dir)*0.5);
        if (millis() - obs_time_start > 1000){
          Robawt.setSteer(0,0);
          curr = 0;
        }
        break;

      //* 90

      case 25: //turn left 90
        Robawt.setSteer(30, -1);
        break;
      
      case 26: //turn right 90
        Robawt.setSteer(30, 1);
        break;
        
      //* 135

      case 27: //left135
        Robawt.setSteer(rpm, -0.8);
        #if debug_with_led
        digitalWrite(ONBOARDLEDPIN, HIGH);
        #endif
        if (millis() - start135Millis > 1500) {
          last135Millis = millis();
          curr = 0;
          left135 = false;
          #if debug_with_led
          digitalWrite(ONBOARDLEDPIN, LOW);
          #endif
        }
        break;
      
      case 28: //right135
        Robawt.setSteer(rpm, 0.8);
        #if debug_with_led
        digitalWrite(ONBOARDLEDPIN, HIGH);
        #endif
        if (millis() - start135Millis > 1500) {
          last135Millis = millis();
          curr = 0;
          right135 = false;
          #if debug_with_led
          digitalWrite(ONBOARDLEDPIN, LOW);
          #endif
        }
        break;

      //* EVAC

      case 39: //initialise evac
        startEvacMillis = millis();
        task = 40;
        curr = 40;
        break;

      case 40: // evac wall track with diagonal lidar
        claw_open();
        evac_setdist = 140 + (millis() - startEvacMillis)/500;
        if (evac_setdist > 600) {evac_setdist = 600;}
        wall_rot = (evac_setdist - fl_dist) * 0.0095;
        Robawt.setSteer(30, wall_rot);
        if (ball_present()) curr = 45;
        break;

      case 41: //turn to ball
        claw_halfclose();
        if (ball_present()) {
          curr = 45; 
        } else {
          Robawt.setSteer(30, rotation);
        }
        break;

      // case 42: //post ball
      //   claw_halfclose();
      //   if (ball_present()) {
      //     curr = 45; //enter pickup
      //   }
      //   Robawt.setSteer(25, 0);
      //   break;

      case 45: //start pickup seq
        claw_close();
        Robawt.setSteer(0,0);
        if (!ball_present()) curr = 40;
        break;
    }

    //* DEBUG PASSED VARIABLES */

    #if debug_curr
    Serial.print("Case: ");
    Serial.println(curr);
    #endif

    #if debug_passed_vars
    Serial.print("Task: ");
    Serial.println(task);
    Serial.print("Rotation: ");
    Serial.println(rotation);
    Serial.print("Rpm: ");
    Serial.println(rpm);
    #endif

    #if debug_teensy_vars
    Serial.print("Evac: ");
    Serial.println(in_evac);
    Serial.print("Left135: ");
    Serial.println(left135);
    Serial.print("Right135: ");
    Serial.println(right135);
    #endif

  } else {

    //* TO MAKE ROBOT STOP */
    Robawt.setSteer(0, 0);
    Robawt.reset();
    claw_open();
    claw_down();
    
    //~ To reset the shady rescue kit code
    startBlueMillis = 0;
    rev_L_dist = 0;
    prev_rotation = 0;
    start_L_dist = 0;

    in_evac = false;
    left135 = false;
    right135 = false;

//    curr = 39; //! force_evac
  }
}
#endif

//* -----------END LOOP-----------*//


//* COMMS FUNCTIONS */

void serialEvent()
{
  while (Serial1.available()) {
    int serialData = Serial1.read();
    Serial.println(serialData);
    if (serialData == 255 || serialData == 254 || serialData == 253 || serialData == 252) {serialState = (int)serialData;}
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
        case 252:
          see_line = (bool)(int)serialData;
          Serial.println(see_line);
          break;
      }
    }
  }
}

void teensyEvent()
{
  int pin1State = digitalRead(TNYPIN1);
  int pin2State = digitalRead(TNYPIN2);
  if (pin1State && pin2State) { //silver 11
    // in_evac = true;
    // curr = 39;
    left135 = false;
    right135 = false;
  } else if (!in_evac && !pin1State && pin2State) { //left 01
    left135 = true;
    right135 = false;
  } else if (!in_evac && pin1State && !pin2State) { //right 10
    left135 = false;
    right135 = true;
  }
  // } else {
  //   // left135 = false;
  //   // right135 = false;
  // }
}

//* LIDAR & MULTIPLEXER FUNCTIONS */

void tcaselect(uint8_t i)  //Multiplexer: TCA9548A
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

void businit(TwoWire *bus, int sdaPin, int sclPin) //I2C for Multiplexer
{
  bus->setSDA(sdaPin);
  bus->setSCL(sclPin);
  bus->begin();
  bus->setClock(400000); 
}

void l0xinit(TwoWire *bus, VL53L0X *sensor, uint8_t i) //Lidar: VL53L0X
{
  tcaselect(i);
  sensor->setTimeout(500);
  while (!sensor->init()) {
    Serial.print(i); 
    Serial.println("L0X failed to initialise");}
  sensor->startContinuous(33);
  // sensor->setBus(bus);
}

void l1xinit(VL53L1X *sensor, uint8_t i) //Lidar: VL53L1X
{
  tcaselect(i);
  sensor->setTimeout(500);
  while (!sensor->init()) {
    Serial.print(i);
    Serial.println("L1X failed to initialise");
    }
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


//* CLAW & SERVO FUNCTIONS */

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

//* EVAC FUNCTIONS */

void send_pi(int i) {
  Serial1.println(i);
}

bool ball_present() {
  return (front_dist - fb_dist > 30 && fb_dist < 95);
}

//* DEBUG LOOPS */

#if debug_servos
void loop() 
{
  if (servos_change) {
    for (int i = Servos::ARM; i != (Servos::S6 + 1); i++) {
      servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
    }
    servos_change = false;
  }
  claw_down();

  if (!digitalRead(SWTPIN)) {
    claw_open();
    // test_all_servos();
  } else {
    claw_close();
    // test_all_servos2();
  }
}
#endif

#if debug_lidars
void loop()
{
  // tcaselect(L_LIDAR);
  // l_dist = l_tof.readRangeContinuousMillimeters();
  tcaselect(FB_LIDAR);
  fb_dist = fb_tof.read();
  tcaselect(FL_LIDAR);
  fl_dist = fl_tof.readRangeContinuousMillimeters();
  // tcaselect(F_LIDAR);
  // front_dist = front_tof.readRangeContinuousMillimeters();
  // Serial.print("left: ");
  // Serial.println(l_dist);
  Serial.print("front below: ");
  Serial.println(fb_dist);
  Serial.print("front left: ");
  Serial.println(fl_dist);
  // Serial.print("front: ");
  // Serial.println(front_dist);
}
#endif

#if debug_switch
void loop()
{
  Serial.print("Switch state");
  Serial.println(digitalRead(SWTPIN));
}
#endif

#if debug_motors
void loop()
{
 Serial.print("Distance (cm): ");
 Serial.print(MotorL.getDist());
 Serial.print("    ");
 Serial.println(MotorR.getDist());
 Serial.print("Angle (deg): ");
 Serial.print(MotorL.getAngle());
 Serial.print("    ");
 Serial.println(MotorR.getAngle());
 Serial.print("Enc Val: ");
 Serial.print(MotorL.getEncVal());
 Serial.print("    ");
 Serial.println(MotorR.getEncVal());
}
#endif

#if debug_pid
void loop()
{
  // Robawt.setSteer(40, 0);
  if (!digitalRead(SWTPIN)) {
    double valL = MotorL.setRpm(40);
    double valR = MotorR.setRpm(40);
  } else {
    double valL = MotorL.setRpm(0);
    double valR = MotorR.setRpm(0);
  }
  Serial.print("Actual rpm: ");
  Serial.print(MotorL.getRpm());
  Serial.print("    ");
  Serial.println(MotorR.getRpm());
  Serial.print(" Output pwm: ");
  Serial.print(valL);
  Serial.print("    ");
  Serial.println(valR);
}
#endif

#if debug_ball || debug_cube
void loop()
{
  if (servos_change) {
    for (int i = Servos::ARM; i != (Servos::S6 + 1); i++) {
      servos[i].writeMicroseconds(pwmangle(servos_angle[i], servos_max_angle[i]));
    }
    servos_change = false;
  }

  if (!digitalRead(SWTPIN)) claw_service_up();
  else claw_down();

  tcaselect(FB_LIDAR);
  fb_dist = fb_tof.read();
  tcaselect(FL_LIDAR);
  fl_dist = fl_tof.readRangeContinuousMillimeters();
  tcaselect(F_LIDAR);
  front_dist = front_tof.readRangeContinuousMillimeters() + 45;

  Serial.print("front below: ");
  Serial.println(fb_dist);
  Serial.print("front left: ");
  Serial.println(fl_dist);
  Serial.print("front: ");
  Serial.println(front_dist);

  Serial.print("See ball: ");
  Serial.println(ball_present());

  if (ball_present()) {
    #if debug_cube
    claw_close_cube();
    #endif
    #if debug_ball
    claw_close();
    #endif
  } else {
    claw_open();
  }
}
#endif

#if debug_teensy_comms
void loop()
{
  teensyEvent();
  Serial.print("In Evac: ");
  Serial.println(in_evac);
  Serial.print("Left 135: ");
  Serial.println(left135);
  Serial.print("Right 135: ");
  Serial.println(right135);

  // //~ To lift the claw to access the teensy
  // servos[SERVOS::ARM].detach();
}
#endif
