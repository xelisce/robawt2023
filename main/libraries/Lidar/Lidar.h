/*
* lidar.h
* contains streamlined lidar L1X and L0X for use on robot using multiplexer
* created 12/02/2023
*/

#include <Arduino.h>
#include "VL53L1X.h"
#include "VL53L0X.h"
#include <Wire.h>

#ifndef LIDAR_H
#define LIDAR_H

//? kinda useless class
class MUX //this assumes that the address is 0x70
{
    public:
        MUX(int sdaPin, int sclPin);
};

class L1X 
{
    public:
        L1X(int pin, int timePeriod=50);
        int readVal();
    
    private:
        VL53L1X _sensor;
        int _pin,
            _value;
};



#endif