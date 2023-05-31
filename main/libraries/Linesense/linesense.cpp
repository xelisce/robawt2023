#include <Arduino.h>
#include "linesense.h"

#define TCAADDR 0x70

struct hsv
{
    int hue;
    int sat;
    int val;
};

struct hsv rgb_to_hsv(float r, float g, float b)
{
    // R, G, B values are divided by 255
    // to change the range from 0..255 to 0..1:
    float h, s, v;
    r /= 255.0;
    g /= 255.0;
    b /= 255.0;
    float cmax = max(r, max(g, b)); // maximum of r, g, b
    float cmin = min(r, min(g, b)); // minimum of r, g, b
    float diff = cmax - cmin;       // diff of cmax and cmin.
    if (cmax == cmin)
        h = 0;
    else if (cmax == r)
        h = fmod((60 * ((g - b) / diff) + 360), 360.0);
    else if (cmax == g)
        h = fmod((60 * ((b - r) / diff) + 120), 360.0);
    else if (cmax == b)
        h = fmod((60 * ((r - g) / diff) + 240), 360.0);
    // if cmax equal zero
    if (cmax == 0)
        s = 0;
    else
        s = (diff / cmax) * 100;
    // compute v
    v = cmax * 100;

    struct hsv curr;
    curr.hue = h;
    curr.sat = s;
    curr.val = v;
    return curr;
}

void tcaselect(uint8_t i)
{
    if (i > 7)
        return;
    Wire1.beginTransmission(TCAADDR);
    Wire1.write(1 << i);
    Wire1.endTransmission();
}

LineSense::LineSense()
{
    Wire1.begin();
    for (uint8_t t = 0; t < 6; t++)
    {
        tcaselect(_tcs_pins[t]);
        tcs[t].begin();
    }
}

void LineSense::update()
{
    tcaselect(0);
    _tcs.getRawData(&_r, &_g, &_b, &_c);
    _backlefthue = rgb_to_hsv(_r, _g, _b).hue;
    _backleftsat = rgb_to_hsv(_r, _g, _b).sat;

    tcaselect(1);
    _tcs.getRawData(&_r, &_g, &_b, &_c);
    _backrighthue = rgb_to_hsv(_r, _g, _b).hue;
    _backrightsat = rgb_to_hsv(_r, _g, _b).sat;

    for (uint8_t t = 2; t < 8; t++)
    {
        int val;
        tcaselect(t);
        _tcs.getRawData(&_r, &_g, &_b, &_c);
        _g = max(_g, (uint16_t)_frontblackreadings[t - 2]);
        _g = min(_g, (uint16_t)_frontwhitereadings[t - 2]);
        _frontreadings[t - 2] = ((double)(_g - _frontblackreadings[t - 2]) / ((double)(_frontwhitereadings[t - 2] - _frontblackreadings[t - 2])));
        _frontsat[t - 2] = rgb_to_hsv(_r, _g, _b).sat;
        // normalise reading
    }
}

void LineSense::debugRaw(int port)
{
    tcaselect(port);
    // _tcs.getRawData(&_r, &_g, &_b, &_c);
    Serial.print("R ");
    Serial.print(_r);
    Serial.print(" G ");
    Serial.print(_g);
    Serial.print(" B ");
    Serial.print(_b);
    Serial.print(" C ");
    Serial.println(_c);
}

void LineSense::calibrateSensors()
{
    int sum = 0;
    long start_time;
    for (uint8_t t = 0; t < 6; t++)
    {
        tcaselect(_tcs_pins[t]);                                
        start_time = millis();
        tcs[t].getRawData(&_r, &_g, &_b, &_c);
        Serial.print(t);
        Serial.print(":");
        Serial.print(millis() - start_time);
        
        Serial.print(" ");
        Serial.print(_c);
        Serial.print(",");
        Serial.print(rgb_to_hsv(_r, _g, _b).hue);
        Serial.print(",");
        Serial.print(rgb_to_hsv(_r, _g, _b).sat);
        Serial.print(",");
        Serial.print(rgb_to_hsv(_r, _g, _b).val);
        Serial.print(" ");
        sum += (rgb_to_hsv(_r, _g, _b).sat);
        Serial.print(" ");
        
    }
    // delay(50);
    // Serial.print(sum);
    Serial.println();
}

// void LineSense::debugRawColor(int color)
// {
//     for (uint8_t t = 0; t < 8; t++)
//     {
//         tcaselect(t);
//         // _tcs.getRawData(&_r, &_g, &_b, &_c);
//         if (color == 0)
//         {
//             Serial.print("R");
//             Serial.print(t);
//             Serial.print(" ");
//             Serial.print(_r);
//         }
//         else if (color == 1)
//         {
//             Serial.print("G");
//             Serial.print(t);
//             Serial.print(" ");
//             Serial.print(_g);
//         }
//         else if (color == 2)
//         {
//             Serial.print("B");
//             Serial.print(t);
//             Serial.print(" ");
//             Serial.print(_b);
//         }
//         else if (color == 4)
//         {
//             Serial.print("C");
//             Serial.print(t);
//             Serial.print(" ");
//             Serial.print(_c);
//         }
//         Serial.print(" ");
//     }
//     Serial.println();
// }

int LineSense::get135()
{
    double black_thresh = 0.25;
    _weights[0] = (_frontreadings[0] < black_thresh);
    _weights[1] = (_frontreadings[1] < black_thresh);
    _weights[2] = (_frontreadings[2] < black_thresh);
    _weights[3] = (_frontreadings[3] < black_thresh);
    _weights[4] = (_frontreadings[4] < black_thresh);
    _weights[5] = (_frontreadings[5] < black_thresh);

    if (_weights[0] && !_weights[1] && (_weights[2] || _weights[3]) && !_weights[4] && !_weights[5])
    {
        return 1;
    }
    else if (!_weights[0] && !_weights[1] && (_weights[2] || _weights[3]) && !_weights[4] && _weights[5])
    {
        return -1;
    }
    else
    {
        return 0;
    }
}

bool LineSense::getevac()
{
    int sum = 0;
    for (uint8_t t = 0; t < 8; t++)
    {
        tcaselect(t);
        _tcs.getRawData(&_r, &_g, &_b, &_c);
        sum += (rgb_to_hsv(_r, _g, _b).sat);
    }
    if (sum < 170)
        return true;
    else
        return false;
}

void LineSense::debugNorm()
{
    Serial.println(_frontreadings[2] - _frontreadings[3]);
}

double LineSense::getBlack(bool forcedtrack)
{
    _weights[0] = 1 * (_frontreadings[0] < 0.5);
    _weights[1] = 1 * (_frontreadings[1] < 0.5);
    _weights[2] = 1 * (_frontreadings[2] < 0.5);
    _weights[3] = -1 * (_frontreadings[3] < 0.5);
    _weights[4] = -1 * (_frontreadings[4] < 0.5);
    _weights[5] = -1 * (_frontreadings[5] < 0.5);

    // for (uint8_t t = 0; t < 6; t++)
    // {
    //     if (_frontsat[t] > 40)
    //     {
    //         _frontreadings[t] = 0.25;
    //         _weights[t] = 0;
    //     }
    // }

    // double finalweight = _weights[0] + _weights[1] + _weights[2] + _weights[3] + _weights[4] + _weights[5];
    double intersection = _weights[0] + _weights[1] + _weights[2] - _weights[3] - _weights[4] - _weights[5];
    // double mid = _weights[1] + _weights[2] - _weights[3] - _weights[4];

    double blackthresh = 0.5;
    // Serial.println(intersection);

    // sat left, right: black 39, 35  to green 64, 60
    // hue left, right 152, 156
    if (!forcedtrack)
    {
        _backleftgreen = _backleftsat > 54 && _backlefthue < 170;
        _backrightgreen = _backrightsat > 50 && _backrighthue < 170;
        Serial.print(",");
        Serial.print(_backleftgreen);
        Serial.print(_backrightgreen);

        if (_backleftgreen && _backrightgreen && intersection >= 4) // double green
            return 50;
        else if (_backleftgreen && intersection >= 2)
            return -40;
        else if (_backrightgreen && intersection >= 2)
            return 40;
    }

    if (_frontreadings[5] < blackthresh && _frontreadings[0] < blackthresh)
        return 30;
    if (_frontreadings[5] < blackthresh && _frontreadings[0] > blackthresh)
        return -20;
    else if (_frontreadings[5] > blackthresh && _frontreadings[0] < blackthresh)
        return 20;

    if (_frontreadings[4] < blackthresh && _frontreadings[1] < blackthresh)
        return 30;
    else if (_frontreadings[4] < blackthresh)
        return -10;
    else if (_frontreadings[1] < blackthresh)
        return 10;
    // if (finalweight >= 2 && _frontreadings[0] < blackthresh)
    //     return 10;
    // else if (finalweight <= -2 && _frontreadings[5] < blackthresh)
    //     return -10;

    // if (finalweight >= 1 && _frontreadings[1] > blackthresh && _frontreadings[0] < blackthresh) // 135
    //     return 40;
    // else if (finalweight <= -1 && _frontreadings[4] > blackthresh && _frontreadings[5] < blackthresh)
    //     return -40;

    _error = _frontreadings[2] - _frontreadings[3];
    _allerrorever += _error;
    _steering = _kp * _error + _kd * (_error - _preverror) + _ki * _allerrorever;
    _preverror = _error;

    return _steering;
}