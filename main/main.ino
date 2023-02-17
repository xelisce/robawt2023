// --------------------------------------
// main.cpp
//
// Main script on robot
// --------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include "VL53L1X.h"
#include "Vroom.h"
// #include "Lidar.h"

#define TCAADDR 0x70

Motor MotorL(13, 12, 19, 18); //M2 swapped
Motor MotorR(10, 11, 16, 17); //M1
Vroom Robawt(&MotorL, &MotorR);
VL53L1X sensor;

void ISRLA() {MotorL.readEncA();}
void ISRLB() {MotorL.readEncB();}
void ISRRA() {MotorR.readEncA();}
void ISRRB() {MotorR.readEncB();}

/* DEPRECATED CODE */
// MUX Mux(&Wire, 20, 21);
// L1X LidarFL(&Wire, 4);
// L0X LidarClaw(6);

const int TX1PIN = 8;
const int RX1PIN = 9;
const int SDAPIN = 20;
const int SCLPIN = 21;
const int SWTPIN = 14;

double rotation;
int serialData;
double rpm = 40;
int caseSwitch = 0;
long prevSwitchMil = millis();
// long oldPosition  = -999;
// int prevEnc = 0;
// int currEnc = 0;
// int rotationDeg = 360;

void serialEvent() 
{
  while (Serial2.available()) {
    serialData = Serial2.read();
    rotation = (double)(serialData-90)/90;
    // Serial.println(rotation);
  }
}

void tcaselect(uint8_t i)
{
    if (i > 7 || i < 0) return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}

void setup() {
  /* USB SERIAL COMMS */
  Serial.begin(9600);
  while (!Serial)
     delay(10);
  Serial.println("USB serial initialised");

  /* PI SERIAL COMMS */
  // Serial2.setRX(RX1PIN);
  // Serial2.setTX(TX1PIN);
  // Serial2.begin(9600); //consider increasing baud
  // while (!Serial2)
  //   delay(10);
  // Serial.println("Pi serial initialised");

  /* MULTIPLEXER */
  // Wire.setSDA(SDAPIN);
  // Wire.setSCL(SCLPIN);
  // Wire.begin();
  // Wire.setClock(400000);

  /* FOR EACH LIDAR (duplicate this block of code for every lidar used and change pin) */
  // tcaselect(4); //lidar pin
  // sensor.setTimeout(timeOut);
  // while (!sensor.init()) {Serial.println("L1X failed to initialise");} 
  // sensor.setDistanceMode(VL53L1X::Medium);
  // sensor.setMeasurementTimingBudget(50000);
  // sensor.startContinuous(timePeriod);

  pinMode(SWTPIN, INPUT);

  attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING); //double check if its change or rising
  attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
  attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
  attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

// L1X LidarFR(4);


void loop() {
  // serialEvent();

  if (digitalRead(SWTPIN)) {

    /* TO MAKE ROBOT LINETRACK */
    // Robawt.setSteer(rpm, rotation);

    /* TO MAKE ROBOT READ LIDARS (for each lidar, duplicate this chunk of code)*/
    // tcaselect(4); //lidar pin
    // int lidar_reading = sensor.read();
    // if (sensor.timeoutOccurred())
    //   Serial.println("SENSOR TIMEOUT");
    // else
    //   Serial.println(lidar_reading);

    /* TO MAKE LEFT MOTOR GO SLOW THEN FAST */
    // switch(caseSwitch) {
    //   case 0:
    //     MotorL.setRpm(40);
    //     if (millis()-prevSwitchMil > 2000) {
    //       caseSwitch = 1;
    //       prevSwitchMil = millis();
    //     }
    //   case 1:
    //     MotorL.setRpm(180);
    //     if (millis()-prevSwitchMil > 2000) {
    //       caseSwitch = 0;
    //       prevSwitchMil = millis();
    //     }
    // }

    /* TO MAKE ROBOT MUV */
    // Robawt.setSteer(rpm, 0);

    /* DEPRECATED CODE */
    // double val = MotorL.setRpm(20);
    // Serial.print("Actual rpm: ");
    // Serial.print(MotorL.getRpm());
    // Serial.print(" Output pwm: ");
    // Serial.println(val);
    // int value = LidarFL.readVal();
    // Serial.println(value);

  } else {

    /* TO MAKE ROBOT STOP */
    // Robawt.setSteer(0, 0);

    /* TO MAKE LEFT MOTOR STOP */
    // MotorL.setRpm(0);
    // MotorL.resetPID();
  }

}