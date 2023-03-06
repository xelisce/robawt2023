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
#include "Claw.h"

#define TCAADDR 0x70
#define L0XADDR 0x29

//* OBJECT INITIALISATIONS */
Motor MotorL(13, 12, 19, 18); //M2 swapped
Motor MotorR(10, 11, 16, 17); //M1
Vroom Robawt(&MotorL, &MotorR);
VL53L0X l_tof, r_tof;
DFServo claw_arm(7, 500, 2000, 180);

//* MOTOR ENCODERS */
void ISRLA() {MotorL.readEncA();}
void ISRLB() {MotorL.readEncB();}
void ISRRA() {MotorR.readEncA();}
void ISRRB() {MotorR.readEncB();}

//* CONSTANTS
const int TX1PIN = 8,
  RX1PIN = 9,
  SDAPIN = 20,
  SCLPIN = 21,
  SWTPIN = 14,


int serialState = 0;
int l_dist = 0;

double rotation = 0;
double rpm = 40;
int task = 0;

void setup() {

  pinMode(SWTPIN, INPUT);

  //* SERVOS */
  // claw_arm.setAngle(0);

  //* USB SERIAL COMMS */
  Serial.begin(9600);
  while (!Serial) delay(10);
  Serial.println("USB serial initialised");

  //* PI SERIAL COMMS */
  // Serial2.setRX(RX1PIN);
  // Serial2.setTX(TX1PIN);
  // Serial2.begin(9600); //consider increasing baud
  // while (!Serial2) delay(10);
  // Serial.println("Pi serial initialised");

  //* MULTIPLEXER */
  businit(&Wire, SDAPIN, SCLPIN);

  l0xinit(&Wire, &l_tof, 3);
  // l0xinit(r_tof, 5);

  //* MOTOR ENCODERS */
  attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
  attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
  attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
  attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}


void loop() {

//  serialEvent();

  if (digitalRead(SWTPIN)) {

    //* ACTUAL CODE
    // switch (task) {

    //   case 0: //normal lt
    //     Robawt.setSteer(rpm, rotation);
    //     break;

    //   case 1: //left gs
    //     Robawt.setSteer(rpm, -0.5);
    //     break;
      
    //   case 2: //right gs
    //     Robawt.setSteer(rpm, 0.5);
    //     break;

    //   case 3: //double gs
    //     Robawt.setSteer(50, 1);
    //     break;

    //   case 4: //red line
    //     Robawt.setSteer(0, 0);
    //     Robawt.reset();
    //     break;


    //   case 5: //moving backwards (blue)
    //     Robawt.setSteer(-rpm, 0);
    //     break;

    // }

    //* READ LIDAR */
    tcaselect(3);
    l_dist = l_tof.readRangeContinuousMillimeters();
    Serial.println(l_dist);

    //* DEBUG PASSED VARIABLES */
    // Serial.print("task: ");
    // Serial.println(task);
    // Serial.print("rotation: ");
    // Serial.println(rotation);
    // Serial.print("rpm: ");
    // Serial.println(rpm);

    //* TEST PID */
    // double val = MotorL.setRpm(20);
    // Serial.print("Actual rpm: ");
    // Serial.print(MotorL.getRpm());
    // Serial.print(" Output pwm: ");
    // Serial.println(val);

  } else {

    // startChangeMillis = millis();

    //* TO MAKE ROBOT STOP */
    Robawt.setSteer(0, 0);
    Robawt.reset();

    //* TO MAKE LEFT MOTOR STOP */
    // MotorL.setRpm(0);
    // MotorL.resetPID();
  }
}


//* FUNCTIONS */

void serialEvent()
{
  while (Serial2.available()) {
    int serialData = Serial2.read();
    if (serialData == 255 || serialData == 254 || serialData == 253) {serialState = (int)serialData;}
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

void tcaselect(uint8_t i)
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