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
    _pastEncVal = 0;
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

//* NO PID
// double Motor::setRpm(double rpm) //* rev min^-1
// {
//     if (rpm >= 0) {
//         analogWrite(_pwmPin1, 0);
//         analogWrite(_pwmPin2, (int)((_neededRpm/210) * 255)); //output
//     } else {
//         analogWrite(_pwmPin1, (int)((_neededRpm/210) * 255));
//         analogWrite(_pwmPin2, 0);
//     }
//     return rpm;
// }



double Motor::getRpm() 
{
    if (_begin && _end) {
        double lastInterval = _end - _begin;
        double nowInterval = micros()-_end; //1/370*60 
        if (lastInterval < nowInterval)
            return 162162/nowInterval; //^ math hack by converting no. of rotations into degrees or sth
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

double Motor::getAngle() {return (double)(_pastEncVal + _encVal % 374) / 374 * 360;}

void Motor::readEncA() 
{
    if(!digitalRead(_encPinB)) {
        _begin = _end;
        _end = micros();
        _encVal ++;
        // _encDir = 1;
    }
}

void Motor::readEncB() 
{
    if (!digitalRead(_encPinA)) {
        _begin = _end;
        _end = micros();
        _encVal --; 
        // _encDir = -1;
    }
}

int Motor::getEncAPin() {return _encPinA;}
int Motor::getEncBPin() {return _encPinB;}
int Motor::getPin1() {return _pwmPin1;}
int Motor::getPin2() {return _pwmPin2;}
int Motor::getEncVal() {return _encVal;}

double Motor::getDist() {return ((double)(_encVal) / 374 * 9 * 3.14159265);} //in cm

void Motor::resetPID() {_motorPID.Reset();}
void Motor::resetEnc() 
{
    _pastEncVal = _encVal % 374;
    _encVal = 0;
}


Vroom::Vroom(Motor *l, Motor *r) 
{
    this->_left = l;
    this->_right = r;
}

void Vroom::setSteer(double rpm, double rotation) 
{
    if (rpm > 100) rpm = 100;
    if (rpm < -100) rpm = -100;
    if (rotation > 1) rotation = 1;
    if (rotation < -1) rotation = -1;
    double slower = rpm*(1-2*fabs(rotation)); //^ change to int when have time?? maybe
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
    _left->resetEnc();
    _right->resetEnc();
}

void Vroom::resetPID()
{
    _left->resetPID();
    _right->resetPID();
}