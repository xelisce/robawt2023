#include <Arduino.h>
#include "Vroom.h"
#include "PID.h"

Motor::Motor(int pin1, int pin2, int encPinA, int encPinB) 
{
    pinMode(pwmPin1, OUTPUT);
    pinMode(pwmPin2, OUTPUT);
    pinMode(encPinA, INPUT_PULLUP);
    pinMode(encPinB, INPUT_PULLUP);
    pwmPin1 = pin1;
    pwmPin2 = pin2;
    this->encPinA = encPinA;
    this->encPinB = encPinB;
    end = millis();
}

void Motor::setSpeed(double speed) 
{
    if (speed > 0) {
        analogWrite(pwmPin1, 0);
        analogWrite(pwmPin2, fabs(speed)*255);
    } else {
        analogWrite(pwmPin1, fabs(speed)*255);
        analogWrite(pwmPin2, 0);
    }
}

void Motor::getSpeed() 
{
    timeInterval = millis() - end;
    if (rotInterval < 500000) {
        real_rpm =  rotInterval / timeInterval;
    } else {
        real_rpm = 0;
    }
    end = millis();
}

void Motor::readEncA() 
{
    if(digitalRead(encPinB)) counter --;
    else counter ++;
}

void Motor::readEncB() 
{
    if (digitalRead(encPinA)) counter ++;
    else counter --;
}

int Motor::getEncAPin() {return encPinA;}
int Motor::getEncBPin() {return encPinB;}



Vroom::Vroom(Motor *l, Motor *r) 
{
    left = l;
    right = r;
}

void Vroom::setSteer(double speed, double rotation) 
{
    if (speed > 1) speed = 1;
    if (speed < -1) speed = -1;
    if (rotation > 1) rotation = 1;
    if (rotation < -1) rotation = -1;
    double slower = speed*(1-2*fabs(rotation));
    if (rotation > 0) {
        left->setSpeed(speed);
        right->setSpeed(slower);
    } else {
        left->setSpeed(slower);
        right->setSpeed(speed);
    }
}