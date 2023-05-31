#include <Arduino.h>
#include <Wire.h>
#include "TCS34725AutoGain.h"
#include "Vroom.h"

TCS34725 tcs1, tcs2;

#define TCAADR 0x70
#define TCSADR 0x29

#define TNYPIN1 9
#define TNYPIN2 8
#define TX0PIN 16
#define RX0PIN 17
#define SDAPIN 4
#define SCLPIN 5
#define SDA1PIN 6
#define SCL1PIN 7
#define SWTPIN 28
#define LEDPIN 3
#define ONBOARDLEDPIN 25


Motor MotorR(13, 12, 1, 0); 
Motor MotorL(10, 11, 18, 19);
Vroom Robawt(&MotorL, &MotorR);

double steerDir;
double left, right;
double leftLux, rightLux;
  
void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }

// int l0x_readings[6] = {200, 0, 0, 0};
const int tca_pins[2] = {2, 4};
// String l0x_labels[4] = {"FRONT: ", "FRONT LEFT: ", "LEFT: ", "RIGHT: "};
// namespace L0X {
//     enum L0X { FRONT, FRONT_LEFT, LEFT, RIGHT };
// }

void setup() 
{
    Serial.begin(9600);
    while (!Serial) delay(10);
    Serial.println("Serial intialised");

    Wire1.setSDA(SDA1PIN);
    Wire1.setSCL(SCL1PIN);
    Wire1.begin();
    Wire1.setClock(400000);
    Serial.println("TCA initialised");

    pinMode(ONBOARDLEDPIN, OUTPUT);

    tcaselect2(2);
    if (!tcs1.attach(&Wire1))
        Serial.println("ERROR: TCS34725 NOT FOUND !!!");
    tcs1.integrationTime(33); // ms
    tcs1.gain(TCS34725::Gain::X01);
    tcaselect2(4);
    if (!tcs2.attach(&Wire1)) Serial.println("ERROR: TCS34725 NOT FOUND !!!");
    tcs2.integrationTime(33); // ms
    tcs2.gain(TCS34725::Gain::X01);

    pinMode(28, INPUT);
    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

void loop() 
{
    digitalWrite(ONBOARDLEDPIN, HIGH);
    if (digitalRead(28)){

        // Robawt.setSteer(30, 0);
        tcaselect2(2);
        if (tcs1.available()) // if current measurement has done
        {
            Serial.println("First TCS selected: ");
            TCS34725::Color color = tcs1.color();
            TCS34725::RawData rawdata = tcs1.raw();
            Serial.print("Color Temp : "); Serial.println(tcs1.colorTemperature());
            Serial.print("Lux        : "); Serial.println(tcs1.lux());
            Serial.print("R          : "); Serial.println(color.r);
            Serial.print("G          : "); Serial.println(color.g);
            Serial.print("B          : "); Serial.println(color.b);
            Serial.print("C          : "); Serial.println(rawdata.c);
            rightLux = tcs1.lux();
        }
        right = (rightLux) / (1298.90 - 226.02);
    
        tcaselect2(4);
        if (tcs2.available()) // if current measurement has done
        {
            Serial.println("Second TCS selected: ");
            TCS34725::Color color = tcs2.color();
            TCS34725::RawData rawdata = tcs2.raw();
            Serial.print("Color Temp : "); Serial.println(tcs2.colorTemperature());
            Serial.print("Lux        : "); Serial.println(tcs2.lux());
            Serial.print("R          : "); Serial.println(color.r);
            Serial.print("G          : "); Serial.println(color.g);
            Serial.print("B          : "); Serial.println(color.b);
            Serial.print("C          : "); Serial.println(rawdata.c);
            leftLux = tcs2.lux();
        }

        left = leftLux / (906.03 - 195.82);
        steerDir = left - right;
        Robawt.setSteer(30, steerDir);
    }

    else {
        Serial.println("No running!");
        Robawt.setSteer(0, 0);
        Robawt.resetPID();
    }
}

void tcaselect(uint8_t i) //I2C Multiplexer: TCA9548A
{
    if (i >= 0 && i <= 7) {
        Wire.beginTransmission(TCAADR);
        Wire.write(1 << i);
        Wire.endTransmission();
    }
}

void tcaselect2(uint8_t i) //I2C Multiplexer: TCA9548A
{
    if (i >= 0 && i <= 7) {
        Wire1.beginTransmission(TCAADR);
        Wire1.write(1 << i);
        Wire1.endTransmission();
    }
}