/* 
* vroom.cpp
* contains drivebase for robot
* created 09/02/2023
* created by xel
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

double Motor::setSpeed(double speed) //* mm s^-1
{return this->setRpm(speed/3.14159)*3.14159;}

double Motor::setRpm(double rpm) //* rev min^-1
{
    _wantedRpm = fabs(rpm); //setpoint
    noInterrupts();
    _realRpm = this->getRpm(); //input
    interrupts();
    _motorPID.Compute();
    if (rpm >= 0) {
        analogWrite(_pwmPin1, 0);
        analogWrite(_pwmPin2, (int)(_neededRpm)); //output
    } else {
        analogWrite(_pwmPin1, (int)(_neededRpm));
        analogWrite(_pwmPin2, 0);
    }
    return _neededRpm;
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

double Motor::getSpeed() //? unneeded function also
{return this->getRpm()*3.14159265;}

double Motor::getRps() //? unneeded function but wtv
{return this->getRpm()/60;}
    

double Motor::getAngle() {return _encVal/370;}

void Motor::readEncA() 
{
    if(!digitalRead(_encPinB)) {
        _begin = _end;
        _end = micros();
        // _encVal ++; //? technically don't need this line
        // _encDir = 1;
        //! didnt account for negative and positive in pid
    }
}

void Motor::readEncB() 
{
    if (!digitalRead(_encPinA)) {
        _begin = _end;
        _end = micros();
        // _encVal --; //? don't need this line either technically
        // _encDir = -1;
        //! didnt account for negative and positive in pid
    }
}

int Motor::getEncAPin() {return _encPinA;}
int Motor::getEncBPin() {return _encPinB;}
int Motor::getPin1() {return _pwmPin1;}
int Motor::getPin2() {return _pwmPin2;}
int Motor::getEncVal() {return _encVal;}

void Motor::resetPID() 
{
    _motorPID.Reset();
}



Vroom::Vroom(Motor *l, Motor *r) 
{
    this->_left = l;
    this->_right = r;
}

void Vroom::setSteer(double rpm, double rotation) 
{
    // if (speed > 1) speed = 1;
    // if (speed < -1) speed = -1;
    if (rpm > 210) rpm = 210;
    if (rpm < -210) rpm = -210;
    if (rotation > 1) rotation = 1;
    if (rotation < -1) rotation = -1;
    double slower = rpm*(1-2*fabs(rotation));
    // if (rotation > 0) {
    //     this->_left->setSpeed(speed);
    //     this->_right->setSpeed(slower);
    // } else {
    //     this->_left->setSpeed(slower);
    //     this->_right->setSpeed(speed);
    // }
    if (rotation > 0) {
        this->_left->setRpm(rpm);
        this->_right->setRpm(slower);
    } else {
        this->_left->setRpm(slower);
        this->_right->setRpm(rpm);
    }
}

void Vroom::reset() 
{
    _left->resetPID();
    _right->resetPID();
}