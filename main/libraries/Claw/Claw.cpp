/* 
* claw.h
* contains all servo movements on particular robot
* created 03/03/23
*/

#include <Arduino.h>
#include <Servo.h>
#include "Claw.h"

DFServo::DFServo(int pin, double minus, double maxus, double range)
{
    _pin = pin;
    _servo.attach(pin);
    _minus = minus;
    _maxus = maxus;
    _range = range;
}

void DFServo::setAngle(double angle)
{
    _angle = angle;
    _servo.writeMicroseconds((_angle / _range) * (_maxus - _minus) + _minus);
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

// bool Claw::available() {return (millis - _lastAction) > 1000;}