/* 
vroom.cpp
contains drivebase for robot
created 09/02/2023
*/
#include <Arduino.h>
#include "PID.h"

#ifndef VROOM_H
#define VROOM_H

class Motor
{
    public:
        Motor(int pin1, int pin2, int encPinA, int encPinB);
        void setSpeed(double speed), //change to calculate rpm after encoders set up
            readEncA(),
            readEncB();
        int getEncAPin(),
            getEncBPin(),
            getEncVal();
        double getSpeed();

    private:
        int _pwmPin1,
            _pwmPin2,
            _encPinA,
            _encPinB,
            _encVal;
        double _timeInterval,
            _rotInterval = 0.090909, //1/11 of a rotation for 
            _end,
            _realrpm;
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