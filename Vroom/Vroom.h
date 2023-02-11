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
            getEncBPin();

    private:
        int pwmPin1,
            pwmPin2,
            encPinA,
            encPinB;
        double timeInterval,
            rotInterval = 0.090909, //1/11 of a rotation for 
            end,
            realrpm;
};

class Vroom
{
    public:
        Vroom(Motor *left, Motor *right);
        void setSteer(double speed, double rotation); //when speed is negative, robot heading switches

    private:
        Motor *left, *right;
};

#endif