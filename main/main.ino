// --------------------------------------
// main.cpp
//
// Main script on robot
// --------------------------------------

#include <Arduino.h>
#include "Vroom.h"
// #include "Lidar.h"

Motor MotorL(13, 12, 19, 18); //M2 swapped
Motor MotorR(10, 11, 16, 17); //M1
Vroom Robawt(&MotorL, &MotorR);

void ISRLA() {MotorL.readEncA();}
void ISRLB() {MotorL.readEncB();}
void ISRRA() {MotorR.readEncA();}
void ISRRB() {MotorR.readEncB();}

// MUX LidarMux(20, 21);
// L1X LidarFR(4);
// L1X LidarFL(5);
// L0X LidarClaw(6);

#define TX1 8
#define RX1 9
#define SWTPIN 14

double rotation;
int serialData;
double rpm = 40;
// int caseSwitch = 0;
// long prevSwitchMil;
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

void setup() {
  /* USB SERIAL COMMS */
  // Serial.begin(9600);
  // while (!Serial)
  //    delay(10);
  // Serial.println("USB serial initialised");

  /* PI SERIAL COMMS */
  Serial2.setRX(RX1);
  Serial2.setTX(TX1);
  Serial2.begin(9600); //consider increasing baud
  while (!Serial2)
    delay(10);
  Serial.println("Pi serial initialised");

  pinMode(SWTPIN, INPUT);

  attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING); //double check if its change or rising
  attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
  attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
  attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}


void loop() {
  serialEvent();
  if (digitalRead(SWTPIN)) {
    Robawt.setSteer(rpm, rotation);
  } else {
    Robawt.setSteer(0, 0);
  }
}