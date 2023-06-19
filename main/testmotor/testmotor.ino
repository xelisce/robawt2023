#include <Arduino.h>
#include <Wire.h>
#include "Vroom.h"
#include <Servo.h>

#define default_serial 1
#define test_pi 0
#define test_teensy 0
#define print 0
#define ONBOARDLEDPIN 25

const int TX1PIN = 8,
  RX1PIN = 9,
  TX0PIN = 16,
  RX0PIN = 17;

bool flipDir;
double steer;

Motor MotorL(12, 13, 1, 0); 
Motor MotorR(11, 10, 19, 18);
  
void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }

void setup() 
{
    flipDir = true;
    #if default_serial
    Serial.begin(9600);
    Serial.println("USB Serial initialised!");
    #endif
    pinMode(28, INPUT);
    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

long start, stop;
void loop() 
{
    if (digitalRead(28)){
        start = millis();
        steer = flipDir ? 30 : -30 
        Serial.println("Still running!");
        Serial.println(MotorL.setRpm(steer));
        stop = millis();
        Serial.print("Loop time: "); Serial.println(stop - start);
        // Serial.println(MotorR.setRpm(30));
    }
    else {
        flipDir = !flipDir //^ flips dir on turning off switch
        Serial.println("Stopped");
        Serial.println(MotorL.setRpm(0));
        // Serial.println("Running backwards!");
        // Serial.println(MotorL.setRpm(-30));
    }
}


// void loop() {
//     digitalWrite(ONBOARDLEDPIN, HIGH);
//     for (int i = 0; i < 255; i++) {
//         digitalWrite(12, LOW);
//         analogWrite(13, i);
//         delay(100);
//     }
//     // digitalWrite(12, LOW);
//     // digitalWrite(13, HIGH);
// }