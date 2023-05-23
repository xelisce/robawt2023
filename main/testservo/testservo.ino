#include <Arduino.h>
#include <Wire.h>
#include "Vroom.h"
#include <Servo.h>
#include "VL53L1X.h"
#include "VL53L0X.h"

// s1, s2, s3, s4, s5, s6
// 27, 26, 22, 21, 20, 2 

Servo servos[6];
const int servos_pin[6] = {27, 26, 22, 21, 20, 2};
const double servos_max_angle[6] = {180, 180, 300, 180, 300, 300};

void setup(){
    pinMode(28, INPUT);
    for (int i = 0; i<6; i++) {
        servos[i].attach(servos_pin[i], 500, 2500);
        servos[i].writeMicroseconds(pwmangle(0, servos_max_angle[i]));
        // servos[i].attach(servos_pin[i], 500, 2500); // (pin, min, max)
    }
}

void loop(){
    if (digitalRead(28)){
        for (int i=0; i<6; i++){
            servos[i].writeMicroseconds(pwmangle(0, servos_max_angle[i]));
        }
        delay(1000);
        for (int i=0; i<6; i++){
            servos[i].writeMicroseconds(pwmangle(servos_max_angle[i], servos_max_angle[i]));
        }
        delay(1000);
    }
    else{
        for (int i=0; i<6; i++){
            servos[i].writeMicroseconds(pwmangle(0, servos_max_angle[i]));
        }
    }
}

int pwmangle(double angle, double max_angle) //Servo PWM
{
    return (int)(angle/max_angle * 2000 + 500);
}
