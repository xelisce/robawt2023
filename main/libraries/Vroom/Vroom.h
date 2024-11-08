/* 
* vroom.h
* contains drivebase for robot
* created 09/02/2023
* includes classes Motor and Vroom (for driving both motors)
* created by xel
*/

#include <Arduino.h>
#include "PID_v1.h"

#ifndef VROOM_H
#define VROOM_H

class Motor
{
    public:
        Motor(int pin1, int pin2, int encPinA, int encPinB);
        double setSpeed(double speed), 
            setRpm(double rpm),
            setVal(double rpm), //^ temporary test function
            getRpm(),
            getSpeed(),
            getRps(),
            getAngle(),
            getDist();
        void readEncA(), readEncB();
            // updatePulseTimings();
        int getEncAPin(), getEncBPin(),
            getPin1(), getPin2(),
            getEncVal();
        void resetPID(),
            resetEnc(),
            resetPWM(); //dont use
            // resetDist();

    private:
        int _pwmPin1, _pwmPin2,
            _encPinA, _encPinB,
            _encDir;
        long int _encVal, _pastEncVal;
        unsigned long long _begin, _end;
        double _neededRpm, _wantedRpm, _realRpm,
            _neededSpeed, _wantedSpeed, _realSpeed,
            _kp = 0.5,
            _ki = 9, //12
            _kd = 0,
            _val, //this is the pwm direct value
            _rpm;
        // bool _dontReset;
        PID _motorPID = PID(&_realRpm, &_neededRpm, &_wantedRpm, _kp, _ki, _kd, DIRECT);
};

class Vroom
{
    public:
        Vroom(Motor *left, Motor *right);
        void setSteer(double rpm, double rotation), //when speed is negative, robot heading switches
            reset(),
            resetPID(),
            stop();

    private:
        Motor *_left, *_right;
};

#endif