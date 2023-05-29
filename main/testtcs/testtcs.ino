#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>


#define TCAADR 0x70
Adafruit_TCS34725 tcs1 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_240MS);
Adafruit_TCS34725 tcs2 = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_240MS);


// Adafruit_TCS34725 tcs = Adafruit_TCS34725();
void setup(){
    Wire1.setSDA(6);
    Wire1.setSCL(7);
    Wire.begin();
    Wire.setClock(400000);

    Serial.begin(9600);
    while (!Serial) delay(10);
    Serial.println("Serial initialised");

    tcaselect2(2);
    if (tcs1.begin(0x29, &Wire1)) {
        Serial.println("Found sense");
    } 
    else {
        Serial.println("No TCS34725 found ... check your connections");
    while (1);
    }

    tcaselect2(4);
    if (tcs2.begin(0x29, &Wire1)) {
        Serial.println("Found sense");
    } 
    else {
        Serial.println("No TCS34725 found ... check your connections");
    while (1);
    }
}


void loop(){
    uint16_t r, g, b, c, colorTemp, lux;
    tcs1.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
    colorTemp = tcs1.calculateColorTemperature_dn40(r, g, b, c);
    lux = tcs1.calculateLux(r, g, b);
    Serial.print("colorTemp: ");
    Serial.println(colorTemp);
    Serial.print("Lux: ");
    Serial.println(lux);
    Serial.println(r);
    Serial.println(g);
    Serial.println(b);
    Serial.println(c);
}

void tcaselect2(uint8_t i) //I2C Multiplexer: TCA9548A
{
    if (i >= 0 && i <= 7) {
        Wire1.beginTransmission(TCAADR);
        Wire1.write(1 << i);
        Wire1.endTransmission();
    }
}