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

#define TCAADDR 0x70
#define WIRE Wire

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
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
}


L1X::L1X(int pin, int timePeriod, int timeOut)
{
    _pin = pin;
    tcaselect(pin);
    sensor.setTimeout(timeOut);
    while (sensor.init()) {delay(10);} //! did not handle the tripping of error in initialization
    sensor.setDistanceMode(VL53L1X::Medium);
    sensor.setMeasurementTimingBudget(50000);
    sensor.startContinuous(timePeriod);
}

int L1X::readVal()
{
    tcaselect(_pin);
    _value = sensor.read();
    if (sensor.timeoutOccurred())
        return -1;
    else
        return _value;
}


L0X::L0X(int pin, int timePeriod, int timeOut)
{
    _pin = pin;
    tcaselect(pin);
    sensor.setTimeout(timeOut);
    while (sensor.init()) {delay(10);} //! also didn't handle here
    sensor.startContinuous(timePeriod);
}

int L0X::readVal()
{
    tcaselect(_pin);
    _value = sensor.readRangeContinuousMillimeters();
    if (sensor.timeoutOccurred())
        return -1;
    else
        return _value;
}