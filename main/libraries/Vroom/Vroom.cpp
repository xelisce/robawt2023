/* 
* vroom.cpp
* contains drivebase for robot
* created 09/02/2023
*/
#include <Arduino.h>
#include "Vroom.h"
#include <PIDController.h>

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
    _motorPID.begin();
    _motorPID.tune(_kp, _ki, _kd);
    _motorPID.limit(-255, 255);
}

double Motor::setSpeed(double speed) //speed entered should be 0-1
{
    _wantedRpm = fabs(speed)*255;
    _neededRpm = _motorPID.compute(_encVal);
    if (_neededRpm > 0) {
        analogWrite(_pwmPin1, 0);
        analogWrite(_pwmPin2, (int)(_neededRpm));
    } else {
        analogWrite(_pwmPin1, (int)(_neededRpm));
        analogWrite(_pwmPin2, 0);
    }
    return _neededRpm;
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