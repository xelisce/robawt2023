/* 
* vroom.h
* contains drivebase for robot
* created 09/02/2023
* includes classes Motor and Vroom (for driving both motors)
*/
#include <Arduino.h>
#include <PIDController.h>

#ifndef VROOM_H
#define VROOM_H

class Motor
{
    public:
        Motor(int pin1, int pin2, int encPinA, int encPinB);
        double setSpeed(double speed);
        void readEncA(), readEncB();
        int getEncAPin(), getEncBPin(),
            getEncVal();

    private:
        int _pwmPin1, _pwmPin2,
            _encPinA, _encPinB,
            _encVal;
        double _neededRpm, _wantedRpm,
            _kp = 0, //! values to be tuned
            _ki = 22,
            _kd = 0;
        PIDController _motorPID;
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