/* 
* vroom.h
* contains drivebase for robot
* created 09/02/2023
* includes classes Motor and Vroom (for driving both motors)
*/
#include <Arduino.h>
#include "PID.h"

#ifndef VROOM_H
#define VROOM_H

class Motor
{
    public:
        Motor(int pin1, int pin2, int encPinA, int encPinB);
        double setSpeed(double speed), getSpeed(),
            readEncA(), readEncB();
        int getEncAPin(), getEncBPin(),
            getEncVal();

    private:
        int _pwmPin1, _pwmPin2,
            _encPinA, _encPinB,
            _encVal;
        double _timeInterval, _rotInterval = 0.090909, //! check if rotInterval is 1/11 & if on change or rising
            _begin, _end,
            _realRpm, _neededRpm, _wantedRpm,
            _kp = 0, //! values to be tuned
            _ki = 22,
            _kd = 0;
        PID _motorPID = PID(&_realRpm, &_neededRpm, &_wantedRpm, _kp, _ki, _kd, DIRECT);
};

class Vroom
{
    public:
        Vroom(Motor *left, Motor *right);
        void setSteer(double speed, double rotation); //when speed is negative, robot heading switches

    private:
        Motor *_left, *_right;
};

#endif