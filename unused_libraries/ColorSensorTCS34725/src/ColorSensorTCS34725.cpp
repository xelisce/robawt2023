
#include "ColorSensorTCS34725.h"

// Manual for library: http://lygte-info.dk/project/ColorSensorTCS34725Library%20UK.html
// This is version 1.00 from 2019-8-1
// By HKJ from lygte-info.dk


//--------------------------------------------------------------------------------
ColorSensorTCS34725::ColorSensorTCS34725(byte sda, byte scl) {
  this->sda = sda;
  this->scl = scl;
  clockHigh();
  dataHigh();
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
}

//--------------------------------------------------------------------------------
boolean ColorSensorTCS34725::begin() {
  pinMode(sda, INPUT);
  pinMode(scl, INPUT);
  dataHigh();
  clockHigh();
  writeReg(0, 0x01);    // Power on
  online = ack && readReg(0) == 0x01;
  if (!online) return false;
  clearInterrupt();
  setIntegrationTime(integrationTime);
  setWaitTime(waitTime);
  setGain(gain);
  delay(3);
  writeReg(0, 0x03 | (waitTime != 0 ? 8 : 0)); // Enabled ADC and optionally wait
  return ack;
}

//--------------------------------------------------------------------------------
byte ColorSensorTCS34725::readReg(byte reg) {
  dataLow();      // Start condition
  clockDelay();
  clockLow();
  writeByte(addr << 1 | 0); // Write address
  writeByte(0x80 | reg);    // Address without auto increment
  clockHigh();
  dataHigh();   // Stop condition
  dataLow();      // Start condition
  clockDelay();
  clockLow();
  writeByte(addr << 1 | 1); // Read address
  byte v = readByte();
  clockHigh();
  dataHigh();   // Stop condition
  return v;
}

//--------------------------------------------------------------------------------
byte ColorSensorTCS34725::readByte(boolean ack) {
  byte w = 0;
  for (byte i = 0; i < 8; i++) {
    clockHigh();
    clockDelay();
    w = (w << 1) | dataRead();
    clockLow();
  }
  // ACK, this is only used when doing multiple reads
  if (ack) dataLow(); else dataHigh();
  clockHigh();
  clockLow();
  dataHigh();
  return w;
}

//--------------------------------------------------------------------------------
RGBC ColorSensorTCS34725::readRGBC() {
  RGBC rgbc;
  dataLow();      // Start condition
  clockDelay();
  clockLow();
  writeByte(addr << 1 | 0); // Write address
  writeByte(0xa0 | 0x14);    // Address with auto increment
  clockHigh();
  dataHigh();   // Stop condition
  clockDelay();
  dataLow();      // Start condition
  clockDelay();
  clockLow();
  writeByte(addr << 1 | 1); // Read address
  rgbc.c = readByte(true) | (readByte(true) << 8);
  rgbc.r = readByte(true) | (readByte(true) << 8);
  rgbc.g = readByte(true) | (readByte(true) << 8);
  rgbc.b = readByte(true) | (readByte(false) << 8);
  clockHigh();
  dataHigh();   // Stop condition
  return rgbc;
}

//--------------------------------------------------------------------------------
void ColorSensorTCS34725::writeReg(byte reg, byte value) {
  dataLow();      // Start condition
  clockDelay();
  clockLow();
  writeByte(addr << 1 | 0); // Write address
  writeByte(reg | 0x80);
  writeByte(value);
  clockHigh();
  dataHigh();   // Stop condition
}

//--------------------------------------------------------------------------------
void ColorSensorTCS34725::writeByte(byte b) {
  for (byte i = 0; i < 8; i++)  {
    if (b & 0x80) dataHigh();
    else dataLow();
    clockHigh();
    b <<= 1;
    clockLow();
  }
  dataHigh();
  clockHigh();
  clockDelay();
  ack = dataRead() == 0;
  clockLow();
  clockDelay();
}

//--------------------------------------------------------------------------------
void ColorSensorTCS34725::setGain(byte gain) {
  if (gain > 3) gain = 3;
  this->gain = gain;
  writeReg(0xf, gain);     // Gain 1x
}

//--------------------------------------------------------------------------------
byte ColorSensorTCS34725::getGainFactor() {
  switch (gain) {
    case 0 : return 1;
    case 1 : return 4;
    case 2 : return 16;
    default: return 60;
  }
}

//--------------------------------------------------------------------------------
void ColorSensorTCS34725::setIntegrationTime(uint16_t ms) {
  if (ms>611) ms=611;
  integrationTime = ms;
  uint16_t v = uint16_t (ms / 2.4 + 0.5);
  writeReg(1, 256 - v);
}

//--------------------------------------------------------------------------------
uint16_t ColorSensorTCS34725::getIntegrationTime() {
  return uint16_t (2.4 * (256 - readReg(1)) + 0.5);
}

//--------------------------------------------------------------------------------
void ColorSensorTCS34725::setWaitTime(uint16_t ms) {
  waitTime = ms;
  if (ms == 0) {  // Disable wait
    writeReg(0, readReg(0) & ~0x08);
    return;
  }
  byte f = ms > 611 ? 12 : 1; // wlong
  writeReg(0x0d, f == 1 ? 0 : 0x02);
  uint16_t v = uint16_t (ms / (2.4 * f) + 0.5);
  if (v > 256) v = 256;
  writeReg(3, 256 - v);
  writeReg(0, readReg(0) | 0x08);
}

//--------------------------------------------------------------------------------
uint16_t ColorSensorTCS34725::getWaitTime() {
  if (!(readReg(0) & 0x08)) return 0; // Wait is disabled
  boolean wlong = readReg(0x0d) & 0x02;
  return uint16_t ((wlong ? 12 : 1) * 2.4 * (256 - readReg(3)) + 0.5);
}

//--------------------------------------------------------------------------------
void ColorSensorTCS34725::clearReady() {
  byte r = readReg(0);
  writeReg(0, r & ~0x02);
  writeReg(0, r);
}

//--------------------------------------------------------------------------------
boolean ColorSensorTCS34725::isReady() {
  return readReg(0x13) & 0x01;
}

//--------------------------------------------------------------------------------
boolean ColorSensorTCS34725::isInterrupt() {
  return readReg(0x13) & 0x10;
}

//--------------------------------------------------------------------------------
void ColorSensorTCS34725::clearInterrupt() {
  dataLow();      // Start condition
  clockDelay();
  clockLow();
  writeByte(addr << 1 | 0); // Write address
  writeByte(0xe6);        // Clear interrupt command
  clockHigh();
  dataHigh();   // Stop condition
}

//--------------------------------------------------------------------------------
void ColorSensorTCS34725::sleep() {
  writeReg(0, 0); // Disable power
}

//--------------------------------------------------------------------------------
void ColorSensorTCS34725::setInterruptLow(uint16_t clearValue) {
  writeReg(0x04, clearValue);
  writeReg(0x05, clearValue >> 8);
}

//--------------------------------------------------------------------------------
void ColorSensorTCS34725::setInterruptHigh(uint16_t clearValue) {
  writeReg(0x06, clearValue);
  writeReg(0x07, clearValue >> 8);
}

void ColorSensorTCS34725::setInterruptFilter(byte samples) {
  if (samples > 3) samples = samples / 5 + 3;
  if (samples > 15) samples = 15;
  writeReg(0x0c, samples);
}

//--------------------------------------------------------------------------------
uint16_t ColorSensorTCS34725::getInterruptLow() {
  return readReg(0x04) | readReg(0x05) << 8;
}

//--------------------------------------------------------------------------------
uint16_t ColorSensorTCS34725::getInterruptHigh() {
  return readReg(0x06) | readReg(0x07) << 8;
}

//--------------------------------------------------------------------------------
byte ColorSensorTCS34725::getInterruptFilter() {
  byte v = readReg(0x0c);
  return (v <= 3) ? v : v * 5 - 15;
}

//--------------------------------------------------------------------------------
void ColorSensorTCS34725::interruptPinEnable(boolean enable) {
  if (enable) writeReg(0, readReg(0) | 0x10);
  else writeReg(0, readReg(0) & ~0x10);
}

//--------------------------------------------------------------------------------
uint16_t ColorSensorTCS34725::maxValue() {
  byte t = (256 - readReg(1));
  return (t >= 64) ? 65535 : t * 1024;
}

//--------------------------------------------------------------------------------
RGBC ColorSensorTCS34725::readRGBCRelative(const RGBC ref) {
  return calcRGBCRelative(ref, readRGBC());
}

//--------------------------------------------------------------------------------
RGBC ColorSensorTCS34725::calcRGBCRelative(const RGBC ref, const RGBC value) {
  RGBC v;
  v.r = (uint16_t) (value.r * 10000L / ref.r);
  v.g = (uint16_t) (value.g * 10000L / ref.g);
  v.b = (uint16_t) (value.b * 10000L / ref.b);
  v.c = (uint16_t) (value.c * 10000L / ref.c);
  return v;
}

//--------------------------------------------------------------------------------
LightValues ColorSensorTCS34725::calcLight(const RGBC values) {
  LightValues lv;
  lv.IR = (values.r + values.g + values.b > values.c) ? (values.r + values.g + values.b - values.c) / 2 : 0;
  uint16_t r = values.r - lv.IR;
  uint16_t g = values.g - lv.IR;
  uint16_t b = values.b - lv.IR;
  lv.lux = (TCS34725_R_Coef * r + TCS34725_G_Coef * g + TCS34725_B_Coef * b) * double(TCS34725_DF) / double(integrationTime) / double(getGainFactor());
  lv.ct = (uint16_t) (TCS34725_CT_Coef * double(b) / double(r)) + TCS34725_CT_Offset;
  return lv;
}

//--------------------------------------------------------------------------------
int ColorSensorTCS34725::calcHue(const RGBC values) {
  uint16_t r = values.r;
  uint16_t g = values.g;
  uint16_t b = values.b;
  uint16_t ma = max(r, max(g, b));
  uint16_t mi = min(r, min(g, b));
  uint16_t limit = ma * COLOR_LIMIT + 5;
  if ((ma - mi) < limit) return -1;
  double d = 60.0 / (ma - mi);

  // Use max and min to split into the 6 segments of the color wheel
  // Then use the last color to estimate the position in that segment.
  // r=0/360, g=120, b=240
  if (r == ma) {  // Red is max
    if (b == mi) {
      // red-green 0-60
      return (g - mi) * d;
    } else {
      // red-blue 360-300
      return 360 - (b - mi) * d;
    }
  } else if (g == ma) { // Green is max
    if (b == mi) {
      // green-red 120-60
      return 120 - (r - mi) * d;
    } else {
      // green-blue 120-180
      return 120 + (b - mi) * d;
    }
  } else { // Blue is max
    if (r == mi) {
      // blue-green 240-180
      return 240 - (g - mi) * d;
    } else {
      // blue-red 240-400
      return 240 + (r - mi) * d;
    }
  }
}

//--------------------------------------------------------------------------------
void ColorSensorTCS34725::setArbitraryGain(uint16_t gain) {
  if (gain >= 60) {
    if (gain > 240) gain = 240;
    setGain(CS_GAIN_60);
    gain = gain * 154 / 60;
  } else if (gain >= 16) {
    setGain(CS_GAIN_16);
    gain = gain * 154 / 16;
  } else if (gain >= 4) {
    setGain(CS_GAIN_4);
    gain = gain * 154 / 4;
  } else {
    if (gain == 0) gain = 1;
    setGain(CS_GAIN_1);
    gain = gain * 154;
  }
  setIntegrationTime(gain);
}

//--------------------------------------------------------------------------------
uint16_t ColorSensorTCS34725::getArbitraryGain() {
  return (getGainFactor() * getIntegrationTime() + 154 / 2) / 154;
}

//--------------------------------------------------------------------------------
// This function uses nearly a kB of memory
void ColorSensorTCS34725::serialPrintAllRegisters() {
  Serial.print(F("0x00 ENABLE: 0b")); Serial.println(readReg(0), 2);
  Serial.print(F("0x01 ATIME:  0x")); Serial.print(readReg(1), 16); Serial.print(F("  time:")); Serial.println(getIntegrationTime());
  Serial.print(F("0x03 WTIME:  0x")); Serial.print(readReg(3), 16); Serial.print(F("  time:")); Serial.println(getWaitTime());
  Serial.print(F("0x04+ AILT:  ")); Serial.println(getInterruptLow());
  Serial.print(F("0x06+ AIHT:  ")); Serial.println(getInterruptHigh());
  Serial.print(F("0x0C PERS:   0b")); Serial.print(readReg(0x0c), 2); Serial.print(F("  count:")); Serial.println(getInterruptFilter());
  Serial.print(F("0x0D CONFIG: 0b")); Serial.println(readReg(0x0d), 2);
  gain = readReg(0x0f);
  Serial.print(F("0x0f CONTROL:0b")); Serial.print(gain, 2); Serial.print(F("  gain: x")); Serial.println(getGainFactor());
  Serial.print(F("0x12 ID:     0x")); Serial.println(readReg(0x12), 16);
  Serial.print(F("0x13 STATUS: 0b")); Serial.println(readReg(0x13), 2);
  RGBC v = readRGBC();
  Serial.print(F("0x14+ CDATA: ")); Serial.println(v.c);
  Serial.print(F("0x16+ RDATA: ")); Serial.println(v.r);
  Serial.print(F("0x18+ GDATA: ")); Serial.println(v.g);
  Serial.print(F("0x1A+ BDATA: ")); Serial.println(v.b);
}
