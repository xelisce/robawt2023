/* 
* claw.h
* contains all servo movements on particular robot
* created 03/03/23
*/

#include <Arduino.h>
#include <Servo.h>
#include "Claw.h"

DFServo::DFServo(int pin)
{
    _pin = pin;
    _servo.attach(pin);
}

void DFServo::setAngle(double angle)
{
    _angle = angle;
}

double DFServo::getAngle() {return _angle;}

Claw::Claw(DFServo *left, DFServo *right, DFServo *lift, DFServo *sort, DFServo *depositLeft, DFServo *depositRight)
{
    _left = left;
    _right = right;
    _lift = lift;
    _sort = sort;
    _depositLeft = depositLeft;
    _depositRight = depositRight;
}