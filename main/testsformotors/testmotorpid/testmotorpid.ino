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

// Motor MotorR(12, 13, 0, 1); 
// Motor MotorL(11, 10, 18, 19);

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
    if (digitalRead(28)){
        // for (int i=0; i<80; i=i+5){
        //     MotorR.setRpm(i); 
        //     MotorL.setRpm(i);  
        //     delay(1000);
        // }
        // for (int i=80; i>0; i=i-5){
        //     MotorR.setRpm(i); 
        //     MotorL.setRpm(i);  
        //     delay(1000);
        // }
        Serial.println("Still running!");
        MotorL.setRpm(30);
        MotorR.setRpm(30);
    }
    else {
        Serial.println("Back running!");
        MotorL.setRpm(-30);
        MotorR.setRpm(-30);
        Serial.println(MotorL.getRpm());
        Serial.println(MotorR.getRpm());
    }
}

// void setup() {
//     pinMode(12, OUTPUT);
//     pinMode(13, OUTPUT);
//     pinMode(ONBOARDLEDPIN, OUTPUT);
// }

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