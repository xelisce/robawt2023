#include <Arduino.h>
#include "Vroom.h"

#define default_serial 1
#define test_pi 0
#define test_teensy 0
#define ONBOARDLEDPIN 25

const int TX1PIN = 8,
  RX1PIN = 9,
  TX0PIN = 16,
  RX0PIN = 17;

Motor MotorL(12, 13, 0, 1); 
Motor MotorR(11, 10, 19, 18);
  
void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }

void setup() 
{
    #if default_serial
    Serial.begin(115200);
    Serial.println("USB Serial initialised!");
    #endif
    pinMode(28, INPUT);
    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

void loop() 
{
    Serial.print("Left: "); Serial.print(MotorL.getRpm());
    Serial.print("Right: "); Serial.println(MotorR.getRpm());
}