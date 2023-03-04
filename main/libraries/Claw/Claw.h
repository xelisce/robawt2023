/* 
* claw.h
* contains all servo movements on particular robot
* created 03/03/23
*/

#include <Arduino.h>
#include <Servo.h>

#ifndef CLAW_H
#define CLAW_H

class DFServo
{
    public:
        DFServo(int pin);
        void setAngle(double angle);
        double getAngle();
    
    private:
        int _pin;
        double _angle;
        Servo _servo;
};

class Claw
{
    public:
        Claw(DFServo *left, 
            DFServo *right, 
            DFServo *lift, 
            DFServo *sort, 
            DFServo *depositLeft, 
            DFServo *depositRight);

    private:
        DFServo *_left,
            *_right,
            *_lift,
            *_sort,
            *_depositLeft,
            *_depositRight;
};


#endif