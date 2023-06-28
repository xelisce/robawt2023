/* 
* tca9548a.h
* contains useful functions for multiplexer tca
* created 25/05/2023
* created by xel
*/

#include <Arduino.h>
#include <Wire.h>

#ifndef TCA_TCS34725_H
#define TCA_TCS34725_H

class TCA9548A 
{
    public:
        TCA9548A(pin_size_t sdaPin, pin_size_t sclPin, TwoWire *bus = &Wire, uint8_t freq = 400000);
        bool begin();
        bool select(uint8_t i);

    private:
        TwoWire *_bus;
        pin_size_t _sdaPin, _sclPin;
        uint8_t _freq;
};

class TCS34725
{
    public:
        TCS34725(uint8_t port, TCA9548A *mux);
        bool begin();

    private:
        TCA9548A *_mux;
        uint8_t _port;
};

#endif