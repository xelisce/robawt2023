
#ifndef __COLORSENSOR__
#define __COLORSENSOR__

#include "Arduino.h"

// Manual for library: http://lygte-info.dk/project/ColorSensorTCS34725Library%20UK.html
// This is version 1.00 from 2019-8-1
// By HKJ from lygte-info.dk


const byte CS_GAIN_1 = 0;
const byte CS_GAIN_4 = 1;
const byte CS_GAIN_16 = 2;
const byte CS_GAIN_60 = 3;
const double COLOR_LIMIT = 0.1;  // calcHue will not work if difference is below this limit

// From application note
const int TCS34725_DF = 310;
const int TCS34725_R_Coef = 0.136;
const int TCS34725_G_Coef = 1.0;
const int TCS34725_B_Coef = -0.444;
const int TCS34725_CT_Coef = 3810;
const int TCS34725_CT_Offset = 1391;

struct LightValues {
  uint16_t IR;
  double lux;
  uint16_t ct;
};

struct RGBC {
  uint16_t r;
  uint16_t g;
  uint16_t b;
  uint16_t c;
};

class ColorSensorTCS34725 {
  private:
    byte scl;
    byte sda;
    byte addr = 0x29;
    boolean ack;
    boolean online=false;    
    byte gain = CS_GAIN_4;
    uint16_t integrationTime = 154;
    uint16_t waitTime = 0;

    void inline clockHigh() {
      pinMode(scl, INPUT);
    }

    void inline clockLow() {
      pinMode(scl, OUTPUT);
    }

    void inline dataHigh() {
      pinMode(sda, INPUT);
    }

    void inline dataLow() {
      pinMode(sda, OUTPUT);
    }

    byte inline dataRead() {
      return digitalRead(sda);
    }

    void inline clockDelay() {
      delayMicroseconds(2);
    }

    void writeByte(byte b);
    byte readByte(boolean ack = false);

  public:
    ColorSensorTCS34725(byte sda, byte scl);
    boolean begin();  // Initialize sensor and start converting
    void sleep();     // Turn sensor off
    boolean isOnline() {
      return online && ack;
    }

    byte readReg(byte reg); // Direct register access
    void writeReg(byte reg, byte value);// Direct register access

    RGBC readRGBC();    // Read all measured values
    uint16_t maxValue();  // Maximum integration value with current settings

    boolean isReady();  // First conversion is ready
    void clearReady();  // This will restart conversion

    // The clear value must be below "low" or above "high" to generate interrupt
    void setInterruptLow(uint16_t clearValue);
    uint16_t getInterruptLow();

    void setInterruptHigh(uint16_t clearValue);
    uint16_t getInterruptHigh();

    // Need samples values before interrupt is trigged
    void setInterruptFilter(byte samples);
    byte getInterruptFilter();

    // Enable output from intr pin
    void interruptPinEnable(boolean enable);

    boolean isInterrupt();
    void clearInterrupt();


    // Higher gain will give higher values
    void setGain(byte gain);    // Select one of the input gains
    byte getGain() {
      return gain;
    };
    byte getGainFactor();// Return the multiplication factor (1, 4, 16, 60) for the current gain
    void setArbitraryGain(uint16_t gain); // Set gain from 1 to 240 by adjusting integrating time and gain, unity is 154ms and 1
    uint16_t getArbitraryGain();  // Calculate gain factor from integrating time and gain

    // Conversion time, any value below 154ms will limit the maximum rgbc values
    // Longer integration time will give higher values, maximum is 614ms
    void setIntegrationTime(uint16_t ms);
    uint16_t getIntegrationTime();

    // Add a delay between conversions
    // Maximum is 7273ms
    void setWaitTime(uint16_t ms);
    uint16_t getWaitTime();


    RGBC calcRGBCRelative(const RGBC ref, const RGBC value);
    RGBC readRGBCRelative(const RGBC ref);  // Measure and adjust compared to a reference
    int calcHue(const RGBC values); // Requires relative values
    LightValues calcLight(const RGBC values);    
    void serialPrintAllRegisters();   // For debug purpose

};

#endif
