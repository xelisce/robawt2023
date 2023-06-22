/* 
* claw.h
* contains all servo movements on particular robot
* created 03/03/23
* includes classes DFServo and Claw
* created by xel
*/


#include <Arduino.h>
#include <Servo.h>

#ifndef CLAW_H
#define CLAW_H

class DFServo
{
    public:
        DFServo(int pin, double minus, double maxus, double range);
        void setAngle(double angle);
        double getAngle();
    
    private:
        int _pin;
        double _angle,
            _minus,
            _maxus,
            _range;
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
        // bool available();
        void open(bool concurrent = false);

    private:
        DFServo *_left,
            *_right,
            *_lift,
            *_sort,
            *_depositLeft,
            *_depositRight;
        // long _lastAction;
};


#endif