#include <Arduino.h>
#include <Wire.h>
#include "VL53L1X.h"
#include "VL53L0X.h" //^ Note the L0X library is blocking --> if have time can rewrite the function
#include "Vroom.h"
#include <Servo.h>

//* OBJECT INITIALISATIONS */
Motor MotorR(13, 12, 19, 18); //M2 swapped
Motor MotorL(10, 11, 1, 0); //M1
Vroom Robawt(&MotorL, &MotorR);

//* MOTOR ENCODERS */
void ISRLA() {MotorL.readEncA();}
void ISRLB() {MotorL.readEncB();}
void ISRRA() {MotorR.readEncA();}
void ISRRB() {MotorR.readEncB();}

bool in_evac = false;
int curr = 0;

//~ 135
bool left135 = false,
  right135 = false;
long last135Millis, start135Millis;

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

void setup() {
  Serial.begin(9600);
  Serial.println("USB serial initialised");

  pinMode(SWTPIN, INPUT_PULLDOWN);
  pinMode(ONBOARDLEDPIN, OUTPUT);
  pinMode(LEDPIN, OUTPUT);
  pinMode(TNYPIN1, INPUT);
  pinMode(TNYPIN2, INPUT);

  //* MOTOR ENCODERS */    
  //^ basically interrupts *every* code to run the encoder code 
  attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
  attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
  attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
  attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

void loop() {
  teensyEvent();
  if (!digitalRead(SWTPIN))
  {
    switch (curr)
    {

      case 0:
        Robawt.setSteer(0, 0);
        Robawt.reset();
        Serial.println("stop");
        break;

      case 1:
        Robawt.setSteer(40, -1);
        Serial.println("left");
        break;

      case 2:
        Robawt.setSteer(40, 1);
        Serial.println("right");
        break;

      case 3:
        Robawt.setSteer(40, 0);
        Serial.println("straight");
        break;
    }
  } else {
    Robawt.setSteer(0, 0);
    Robawt.reset();
  }
}

void teensyEvent()
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