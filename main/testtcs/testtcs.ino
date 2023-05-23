#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_TCS34725.h>


Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_240MS);

// Adafruit_TCS34725 tcs = Adafruit_TCS34725();
void setup(){
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();
    Wire.setClock(400000);

    Serial.begin(9600);

    if (tcs.begin()) {
        Serial.println("Found sense");
    } 
    else {
        Serial.println("No TCS34725 found ... check your connections");
    while (1);
  }
}


void loop(){
    uint16_t r, g, b, c, colorTemp, lux;
    tcs.getRawData(&r, &g, &b, &c);
  // colorTemp = tcs.calculateColorTemperature(r, g, b);
    colorTemp = tcs.calculateColorTemperature_dn40(r, g, b, c);
    lux = tcs.calculateLux(r, g, b);
    Serial.print("colorTemp: ");
    Serial.println(colorTemp);
    Serial.print("Lux: ");
    Serial.println(lux);
    Serial.println(r);
    Serial.println(g);
    Serial.println(b);
    Serial.println(c);
}