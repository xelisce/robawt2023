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

MUX::MUX(TwoWire *bus, int sdaPin, int sclPin)
{
    _bus = bus;
    _bus->setSDA(sdaPin);
    _bus->setSCL(sclPin);
    _bus->begin();
    _bus->setClock(400000);
}

void MUX::tcaselect(uint8_t i)
{
    if (i > 7 || i < 0) return;
    _bus->beginTransmission(TCAADDR);
    _bus->write(1 << i);
    _bus->endTransmission();
}


L1X::L1X(MUX *mux, int pin, int timePeriod, int timeOut)
{
    _pin = pin;
    _mux = mux;
    _mux->tcaselect(pin);

    _sensor.init();
    _sensor.setDistanceMode(VL53L1X::Medium);
    _sensor.setMeasurementTimingBudget(50000);
    _sensor.startContinuous(timePeriod);
    // _sensor.setBus(_mux);
//     sensor.setTimeout(timeOut);
    // while (!sensor.init()) {Serial.println("L1X failed to initialise");} 
//     sensor.setDistanceMode(VL53L1X::Medium);
//     sensor.setMeasurementTimingBudget(50000);
//     sensor.startContinuous(timePeriod);
}

int L1X::readVal()
{
    _mux->tcaselect(_pin);
    _value = _sensor.read();
    if (_sensor.timeoutOccurred())
        return -1;
    else
        return _value;
}


// L0X::L0X(int pin, int timePeriod, int timeOut)
// {
//     _pin = pin;
//     tcaselect(pin);
//     sensor.setTimeout(timeOut);
//     while (!sensor.init()) {Serial.println("L1X failed to initialise");}
//     sensor.startContinuous(timePeriod);
// }

// int L0X::readVal()
// {
//     tcaselect(_pin);
//     _value = sensor.readRangeContinuousMillimeters();
//     if (sensor.timeoutOccurred())
//         return -1;
//     else
//         return _value;
// }