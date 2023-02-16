/* 
* vroom.cpp
* contains drivebase for robot
* created 09/02/2023
*/
#include <Arduino.h>
#include "Vroom.h"
#include "PID_v1.h"

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
    _motorPID.SetMode(AUTOMATIC);
    // _begin = micros();
    // _end = micros();
    //^ last change before test
}

double Motor::setSpeed(double speed) //* m s^-1
{
    _wantedSpeed = speed; 
    noInterrupts();
    _realSpeed = this->getSpeed();
    interrupts();
    _motorPID.Compute();
    if (_neededSpeed > 0) {
        analogWrite(_pwmPin1, 0);
        analogWrite(_pwmPin2, (int)(_neededSpeed));
    } else {
        analogWrite(_pwmPin1, (int)(_neededSpeed));
        analogWrite(_pwmPin2, 0);
    }
    return _realSpeed;
}

double Motor::setRpm(double rpm) //* rev min^-1
{
    _wantedRpm = rpm;
    noInterrupts();
    _realRpm = this->getRpm();
    interrupts();
    _motorPID.Compute();
    if (_neededRpm > 0) {
        analogWrite(_pwmPin1, 0);
        analogWrite(_pwmPin2, (int)(_neededRpm));
    } else {
        analogWrite(_pwmPin1, (int)(_neededRpm));
        analogWrite(_pwmPin2, 0);
    }
    return _realRpm;
}

double Motor::getSpeed()
{
    if (_begin && _end) {
        double lastInterval = _end - _begin;
        double nowInterval = micros()-_end;
        if (lastInterval < nowInterval)
            return 509.447/nowInterval;
        else
            return 509.447/lastInterval;
    } else {
        return 0;
    }
}

double Motor::getRpm() 
{
    if (_begin && _end) {
        double lastInterval = _end - _begin;
        double nowInterval = micros()-_end; //1/370*60 
        if (lastInterval < nowInterval)
            return 162162/nowInterval;
        else
            return 162162/lastInterval;
    } else {
        return 0;
    }
}

double Motor::getRps() //? unneeded function but wtv
{
    if (_begin && _end) {
        double lastInterval = _end - _begin;
        double nowInterval = micros()-_end; //1/370*60 
        if (lastInterval < nowInterval)
            return 2702.7/nowInterval;
        else
            return 2702.7/lastInterval;
    } else {
        return 0;
    }
}

double Motor::getAngle() {return _encVal/370;}

void Motor::readEncA() 
{
    if(!digitalRead(_encPinB)) {
        _begin = _end;
        _end = micros();
        _encVal ++; //? technically don't need this line
    }
}

void Motor::readEncB() 
{
    if (!digitalRead(_encPinA)) {
        _begin = _end;
        _end = micros();
        _encVal --;
    }
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