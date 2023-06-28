#include <Arduino.h>
#include "linesense.h"
#include <Wire.h>

#define TCAADR 0x70
#define TCSADR 0x29

#define SDA1PIN 6
#define SCL1PIN 7


void LineSense::InitSense(uint8_t start, uint8_t end){
    Wire1.setSDA(SDA1PIN);
    Wire1.setSCL(SCL1PIN);
    Wire1.setClock(400000);
    Wire1.begin();
    for (uint8_t i = start; i < end + 1; i++){
        tcaselect2(tcs_pins[i]);
        while (!tcs[i].begin(TCSADR, &Wire1)) { Serial.println("ERROR: TCS34725 No. "); Serial.print(i); Serial.println(" NOT FOUND!"); }
    }
}

void LineSense::update(uint8_t start, uint8_t end) {
    for (int i = start; i < end + 1; i++){
        tcaselect2(tcs_pins[i]);
        tcs[i].getRawData(&tcsSensors[i].r, &tcsSensors[i].g, &tcsSensors[i].b, &tcsSensors[i].c);
        rgb_to_hsv(&tcsSensors[i]);
    }
}

void LineSense::debugRaw(uint8_t start, uint8_t end){
    for (int i = start; i < end + 1; i++){
        tcaselect2(tcs_pins[i]);
        Serial.print("Sensor "); Serial.print(i); Serial.print("    ");
        Serial.print("R: "); Serial.print(tcsSensors[i].r); Serial.print(" ");
        Serial.print("G: "); Serial.print(tcsSensors[i].g); Serial.print(" ");
        Serial.print("B: "); Serial.print(tcsSensors[i].b); Serial.print(" ");
        Serial.print("C: "); Serial.print(tcsSensors[i].c); Serial.print(" ");
        Serial.print("H: "); Serial.print(tcsSensors[i].hue, DEC); Serial.print(" ");
        Serial.print("S: "); Serial.print(tcsSensors[i].sat, DEC); Serial.print(" ");
        Serial.print("V: "); Serial.print(tcsSensors[i].val, DEC); Serial.print(" ");
    }

    Serial.println();
    for (uint8_t i = 1; i <end; i++){
        Serial.print(i);
        Serial.print("Is black: ");
        Serial.print(isBlack(i));
    }
}

void rgb_to_hsv(LineSense::tcsSensor* currentTCS){
    // R, G, B values are divided by 255
    // to change the range from 0..255 to 0..1:
    float r = currentTCS->r / 255.0;
    float g = currentTCS->g / 255.0;
    float b = currentTCS->b / 255.0;
    float cmax = max(r, max(g, b)); // maximum of r, g, b
    float cmin = min(r, min(g, b)); // minimum of r, g, b
    float diff = cmax - cmin;       // diff of cmax and cmin.
    if (cmax == cmin)
        currentTCS->hue = 0;
    else if (cmax == r)
        currentTCS->hue = fmod((60 * ((g - b) / diff) + 360), 360.0);
    else if (cmax == g)
        currentTCS->hue = fmod((60 * ((b - r) / diff) + 120), 360.0);
    else if (cmax == b)
        currentTCS->hue = fmod((60 * ((r - g) / diff) + 240), 360.0);
    // if cmax equal zero
    if (cmax == 0)
        currentTCS->sat = 0;
    else
        currentTCS->sat = (diff / cmax) * 100;
    // compute v
    currentTCS->val = cmax * 100;
}

void tcaselect2(uint8_t i) //I2C Bottom Multiplexer: TCA9548A
{
    if (i > 0 && i < 7) {
        Wire1.beginTransmission(TCAADR);
        Wire1.write(1 << i);
        Wire1.endTransmission();
    }
}

bool isBlack(int i)
{
    if (!(LineSense::tcsSensors[i].green) && !(LineSense::tcsSensors[i].red) && grayPercent(i) < 0.5) { return true; }
    else { return false; }
}