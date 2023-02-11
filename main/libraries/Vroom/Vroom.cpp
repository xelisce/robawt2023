/* 
vroom.cpp
contains drivebase for robot
created 09/02/2023
*/
#include <Arduino.h>
#include "Vroom.h"
#include "PID.h"

Motor::Motor(int pin1, int pin2, int encPinA, int encPinB) 
{
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(encPinA, INPUT_PULLUP);
    pinMode(encPinB, INPUT_PULLUP);
    _pwmPin1 = pin1;
    _pwmPin2 = pin2;
    this->_encPinA = encPinA;
    this->_encPinB = encPinB;
    _end = millis();
}

void Motor::setSpeed(double speed) 
{
    if (speed > 0) {
        analogWrite(_pwmPin1, 0);
        analogWrite(_pwmPin2, fabs(speed)*255);
    } else {
        analogWrite(_pwmPin1, fabs(speed)*255);
        analogWrite(_pwmPin2, 0);
    }
}

double Motor::getSpeed() 
{
    _timeInterval = millis() - _end;
    if (_rotInterval < 500000) {
        _realrpm =  _rotInterval / _timeInterval;
    } else {
        _realrpm = 0;
    }
    _end = millis();
    return _realrpm;
}

void Motor::readEncA() 
{
    if(digitalRead(_encPinB)) _encVal --;
    else _encVal ++;
}

void Motor::readEncB() 
{
    if (digitalRead(_encPinA)) _encVal ++;
    else _encVal --;
}

int Motor::getEncAPin() {return _encPinA;}
int Motor::getEncBPin() {return _encPinB;}
int Motor::getEncVal() {return _encVal;}



Vroom::Vroom(Motor *l, Motor *r) 
{
    this->_left = l;
    this->_right = r;
}

void Vroom::setSteer(double speed, double rotation) 
{
    if (speed > 1) speed = 1;
    if (speed < -1) speed = -1;
    if (rotation > 1) rotation = 1;
    if (rotation < -1) rotation = -1;
    double slower = speed*(1-2*fabs(rotation));
    if (rotation > 0) {
        this->_left->setSpeed(speed);
        this->_right->setSpeed(slower);
    } else {
        this->_left->setSpeed(slower);
        this->_right->setSpeed(speed);
    }
}