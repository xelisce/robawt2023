/* 
* linesense.cpp
* contains library for tcssensors
* created 26/6/23
* Reorganised by Dom
*/

#include <Adafruit_TCS34725.h>
#include <Arduino.h>

#ifndef linesense_h
#define linesense_h

class LineSense {

    public:
        void InitSense(uint8_t start=0, uint8_t end=6);
        void update(uint8_t start=0, uint8_t end=6);
        void debugRaw(uint8_t start=0, uint8_t end=6);
        struct tcsSensor
        {
            uint16_t r;
            uint16_t g;
            uint16_t b;
            uint16_t c;
            int hue;
            int sat;
            int val;
            bool green = false;
            bool red = false;
            bool black = false;
            bool silver = false;
        };
        struct tcsSensor tcsSensors[6];
    private:
        const int tcsNum = 6;
        Adafruit_TCS34725 tcs[6] = {Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X),
                           Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X)};
        const int tcs_pins[tcsNum] = {5, 4, 2, 1, 6, 0}; //from left to right, then second row with robot facing forward

        const int tcs_black[tcsNum] = {0, 104, 124, 0, 0, 0};
        const int tcs_white[tcsNum] = {0, 331, 410, 0, 0, 0};
        const int tcs_lgreen[tcsNum] = {0, 0, 0, 0, 0, 0};
        const int tcs_ugreen[tcsNum] = {0, 0, 0, 0, 0, 0};
        const int tcs_lred1[tcsNum] = {0, 0, 0, 0, 0, 0};
        const int tcs_ured1[tcsNum] = {0, 0, 0, 0, 0, 0};
        const int tcs_lred2[tcsNum] = {0, 0, 0, 0, 0, 0};
        const int tcs_ured2[tcsNum] = {0, 0, 0, 0, 0, 0};

};

#endif