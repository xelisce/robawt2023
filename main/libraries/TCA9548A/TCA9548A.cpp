/* 
* tca9548a.cpp
* contains useful functions for multiplexer tca
* created 25/05/2023
* created by xel
*/

#include "TCA9548A.h"

#define TCAADR 0x70

TCA9548A::TCA9548A(pin_size_t sdaPin, pin_size_t sclPin, TwoWire *bus, uint8_t freq) 
{
    _bus = bus;
    _freq = freq;
    _sdaPin = sdaPin;
    _sclPin = sclPin;
}

bool TCA9548A::begin()
{
    _bus->setSDA(_sdaPin);
    _bus->setSCL(_sclPin);
    _bus->setClock(_freq);
    _bus->begin();
    return true;
}

bool TCA9548A::select(uint8_t i)
{
    if (i < 0 || i > 7) return false;
    _bus->beginTransmission(TCAADR);
    _bus->write(1 << i);
    _bus->endTransmission();
    return true;
}