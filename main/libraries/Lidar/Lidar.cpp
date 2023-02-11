/*
* lidar.cpp
* contains streamlined lidar L1X and L0X for use on robot using multiplexer
* created 12/02/2023
*/

#include <Arduino.h>
#include "VL53L1X.h"
#include "VL53L0X.h"
#include <Wire.h>
#include "Lidar.h"


MUX::MUX(int sdaPin, int sclPin)
{
    Wire.setSDA(sdaPin);
    Wire.setSCL(sclPin);
    Wire.begin();
    Wire.setClock(400000);
}

void tcaselect(uint8_t i)
{
    if (i > 7 || i < 0) return;
    Wire.beginTransmission(0x70);
    Wire.write(1 << i);
    Wire.endTransmission();
}

L1X::L1X(int pin, int timePeriod)
{
    _pin = pin;
    _sensor.setTimeout(500);
    _sensor.init(); //! did not handle the tripping of error in initialization
    _sensor.setDistanceMode(VL53L1X::Medium);
    _sensor.setMeasurementTimingBudget(50000);
    _sensor.startContinuous(timePeriod);
}

int L1X::readVal()
{
    tcaselect(_pin);
    _value = _sensor.read();
    if (_sensor.timeoutOccurred())
        return _value;
    else
        return -1;
}