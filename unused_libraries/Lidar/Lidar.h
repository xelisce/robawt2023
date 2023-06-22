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

// class MUX
// {
//     public:
//         MUX(TwoWire *bus, int sdaPin, int sclPin);
//         void tcaselect(uint8_t i);

//     private:
//         TwoWire *_bus;
// };

// class L1X 
// {
//     public:
//         L1X(MUX *mux, int pin, int timePeriod=50, int timeOut=500);
//         int readVal();
    
//     private:
//         VL53L1X _sensor;
//         MUX *_mux;
//         int _pin,
//             _value;
// };

class L0X
{
    public:
        L0X(TwoWire *bus, int pin, int timePeriod = 50, int timeOut=500);
        void tcaselect();
        int readVal();

    private:
        TwoWire *_bus;
        VL53L0X sensor;
        int _pin,
            _value;
};



#endif