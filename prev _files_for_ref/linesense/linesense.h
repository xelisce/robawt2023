#include "Wire.h"
#include "Adafruit_TCS34725.h"
#include <Arduino.h>

#ifndef linesense_h
#define linesense_h

#define BLACK_THRESHOLD 0.5
#define GREEN_HUE_THRESHOLD 0

class LineSense
{
public:
    LineSense();
    void debugRaw(int port);
    // void debugRawColor(int color);
    void debugNorm();
    void update();
    void calibrateSensors();
    bool getevac();
    double getBlack(bool forcedtrack = false);
    int get135();
    // int[2] getGreen();

private:
    double _error, _preverror, _preverrortime, _steering, _kp = 0.3, _ki = 0.001, _kd = 0, _allerrorever;
    uint16_t _r, _b, _g, _c;
    // int _frontblackreadings[6] = {20, 25, 21, 25, 26, 23}; // val
    // int _frontwhitereadings[6] = {95, 133, 149, 147, 131, 109};
    int _frontblackreadings[6] = {39, 65, 58, 57, 59, 42}; // green value
    int _frontwhitereadings[6] = {234, 334, 402, 402, 328, 276};
    // int _frontsilverthresh[6] = {19, 20, 21, 19, 19, 19}
    // double _frontgreenreadings[6] = {1.7, 1.8, 1.8, 1.9, 1.8, 1.8}; //sat
    // green: 2.11, 2.22, 2.33, 2.48, 2.20, 2.45
    // blacc: 1.32, 1.47, 1,45, 1.60, 1.44, 1.44
    // white:
    double _weights[6] = {2, 1.5, 1, -1, -1.5, -2};
    double _frontreadings[6];
    double _frontsat[6];
    double _backlefthue, _backrighthue, _backleftsat, _backrightsat;
    int _backleftgreen, _backrightgreen;
    Adafruit_TCS34725 _tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_154MS, TCS34725_GAIN_1X);
};

#endif