// --------------------------------------
// main.cpp
//
// Main script on robot
// --------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include "VL53L1X.h"
#include "VL53L0X.h"
#include "Vroom.h"
// #include "Lidar.h"
// #include "Claw.h"

#define TCAADDR 0x70

Motor MotorL(13, 12, 19, 18); //M2 swapped
Motor MotorR(10, 11, 16, 17); //M1
Vroom Robawt(&MotorL, &MotorR);
VL53L0X l_tof, r_tof;

void ISRLA() {MotorL.readEncA();}
void ISRLB() {MotorL.readEncB();}
void ISRRA() {MotorR.readEncA();}
void ISRRB() {MotorR.readEncB();}

/* DEPRECATED CODE */
// MUX Mux(&Wire, 20, 21);
// L1X LidarFL(&Mux, 4);
// L0X LidarClaw(6);

const int TX1PIN = 8;
const int RX1PIN = 9;
const int SDAPIN = 20;
const int SCLPIN = 21;
const int SWTPIN = 14;

double rotation;
int serialState = 0;
double rpm;
int caseSwitch = 0;
long prevSwitchMil = millis();
long startChangeMillis;
int task;
int curr = 0;
// long oldPosition  = -999;
// int prevEnc = 0;
// int currEnc = 0;
// int rotationDeg = 360;

void setup() {
  /* USB SERIAL COMMS */
  Serial.begin(9600);
  while (!Serial)
     delay(10);
  Serial.println("USB serial initialised");

  /* PI SERIAL COMMS */
  Serial2.setRX(RX1PIN);
  Serial2.setTX(TX1PIN);
  Serial2.begin(9600); //consider increasing baud
  while (!Serial2) delay(10);
  Serial.println("Pi serial initialised");

  // l0xinit(l_tof, 4);
  // l0xinit(r_tof, 5);

  pinMode(SWTPIN, INPUT);

  attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
  attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
  attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
  attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

// L1X LidarFR(4);


void loop() {
  // while (Serial2.available())
  // {
  //   processIncomingByte(Serial2.read());
  // }

  serialEvent();

  if (digitalRead(SWTPIN)) {

    //* DEBUG PASSED VARIABLES */
    
    Serial.print("task: ");
    Serial.println(task);
    Serial.print("rotation: ");
    Serial.println(rotation);
    Serial.print("rpm: ");
    Serial.println(rpm);

    switch (task) {

      case 0: //normal lt
        // if (task == 0) {
        Robawt.setSteer(rpm, rotation);
        // } else {
          // curr = task;
        //   startChangeMillis = millis();
        // }
        break;

      case 1: //left gs
        // if (millis() - startChangeMillis < 2000) {
        Robawt.setSteer(rpm, -0.5);
        // analogWrite(MotorR.getPin1(), 70);
        // analogWrite(MotorR.getPin2(), 0);
        // analogWrite(MotorL.getPin1(), 0);
        // analogWrite(MotorL.getPin2(), 0);
        // } else if (millis() - startChangeMillis < 4000) {
          // Robawt.setSteer(60, -0.3);
        // } else {
        //   curr = 0;
        // }
        break;
      
      case 2: //right gs
        // if (millis() - startChangeMillis < 2000) {
        Robawt.setSteer(rpm, 0.5);
        // analogWrite(MotorL.getPin1(), 70);
        // analogWrite(MotorL.getPin2(), 0);
        // analogWrite(MotorR.getPin1(), 0);
        // analogWrite(MotorR.getPin2(), 0);
        // } else {
        //   curr = 0;
        // }
        break;

      case 3: //double gs
        // if (millis() - startChangeMillis < 1500) {
        Robawt.setSteer(60, 1);
        // } else {
        //   curr = 0;
        // }
        break;

      case 4: //red line
        Robawt.setSteer(0, 0);
        Robawt.reset();

      case 5: //moving backwards (blue)
        Robawt.setSteer(-rpm, 0);
        break;

    }




    /* TO MAKE ROBOT LINETRACK */
    // if (task != 0)
    // {
    //   Robawt.setSteer(0, 0);
    // } else {
    //   Robawt.setSteer(60, rotation);
    // }

    /* TO MAKE ROBOT READ LIDARS (for each lidar, duplicate this chunk of code)*/
    // tcaselect(4); //lidar pin
    // int lidar_reading = sensor.read();
    // if (sensor.timeoutOccurred())
    //   Serial.println("SENSOR TIMEOUT");
    // else
    //   Serial.println(lidar_reading);

    /* TO MAKE ROBOT MUV */
    // Robawt.setSteer(50, 0);

    /* TO MAKE ROBOT TURN FOR GS */
    // switch (curr) {
    //   case 0:
    //     Robawt.setSteer(60, 0.4);
    //     if (millis()-startChangeMillis > 2000) {
    //       curr = 1;
    //     }

    //   case 1:
    //     Robawt.setSteer(0, 0);

    // }

    /* DEPRECATED CODE */
    // double val = MotorL.setRpm(20);
    // Serial.print("Actual rpm: ");
    // Serial.print(MotorL.getRpm());
    // Serial.print(" Output pwm: ");
    // Serial.println(val);
    // int value = LidarFL.readVal();
    // Serial.println(value);

  } else {

    // startChangeMillis = millis();
    // curr = 0;

    /* TO MAKE ROBOT STOP */
    Robawt.setSteer(0, 0);
    Robawt.reset();

    /* TO MAKE LEFT MOTOR STOP */
    // MotorL.setRpm(0);
    // MotorL.resetPID();
  }
}




void serialEvent()
{
  while (Serial2.available()) {
    int serialData = Serial2.read();
    if (serialData == 255) {serialState = 255;}
    else if (serialData == 254) {serialState = 254;}
    else if (serialData == 253) {serialState = 253;}
    else if (serialState == 255) {rotation = ((double)(serialData)-90)/90;}
    else if (serialState == 254) {rpm = (int)serialData;}
    else if (serialState == 253) {task = (int)serialData;}
  }
}

void tcaselect(uint8_t i)
{
  if (i < 0 || i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void l0xinit(VL53L0X &sensor, uint8_t pin)
{
  tcaselect(pin);
  sensor.setTimeout(500);
  while (!sensor.init()) {Serial.println("L0X failed to initialise");}
  sensor.startContinuous(50);
}

int l0xread(VL53L0X &sensor, uint8_t pin)
{
  tcaselect(pin);
  int value = sensor.readRangeContinuousMillimeters();
  if (sensor.timeoutOccurred()) return -1;
  else return value;
}

void l1xinit(VL53L1X &sensor, uint8_t pin)
{
  tcaselect(pin);
  sensor.setTimeout(500);
  while (!sensor.init()) {Serial.println("L1X failed to initialise");}
  sensor.setDistanceMode(VL53L1X::Medium);
  sensor.startContinuous(50);
}

int l1xread(VL53L1X &sensor, uint8_t pin)
{
  tcaselect(pin);
  int value = sensor.read();
  if (sensor.timeoutOccurred()) return -1;
  else return value;
}

void businit(TwoWire *bus, int sdaPin, int sclPin)
{
  bus->setSDA(sdaPin);
  bus->setSCL(sclPin);
  bus->begin();
  bus->setClock(400000); 
}