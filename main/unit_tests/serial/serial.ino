#include <Arduino.h>

//* PINS
#define LEDPIN 3
#define SDAPIN 4
#define SCLPIN 5
#define SDA1PIN 6
#define SCL1PIN 7
#define TX0PIN 16
#define RX0PIN 17
#define PICOLEDPIN 25
#define SWTPIN 28

unsigned long long lastSerialPiSendMillis;

void setup()
{
    //^ USB SERIAL COMMS
    Serial.begin(9600);
    Serial.println("USB serial initialised");

    //^ PI SERIAL COMMS
    Serial1.setRX(RX0PIN); 
    Serial1.setTX(TX0PIN);
    Serial1.begin(115200);
    while (!Serial1) delay(10); 
    Serial.println("Pi serial initialised");
}

int counter = 0;

void loop()
{
    send_pi(counter);
    counter++;
    delay(2000);
}

void send_pi(int i) //Pico to pi serial
{
    if (millis() - lastSerialPiSendMillis > 100) { 
        Serial1.println(i);
        lastSerialPiSendMillis = millis();
    }
}