#include <Arduino.h>
#include <Vroom.h>


#define TCAADR 0x70
#define L0XADR 0x29
#define L1XADR 0x29

#define LEDPIN 3
#define SDAPIN 4
#define SCLPIN 5
#define SDA1PIN 6
#define SCL1PIN 7
#define TX0PIN 16
#define RX0PIN 17
#define PICOLEDPIN 25
#define SWTPIN 28


Motor MotorL(12, 13, 1, 0); 
Motor MotorR(11, 10, 19, 18);
Vroom Robawt(&MotorL, &MotorR);

void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }


void setup(){
    pinMode(SWTPIN, INPUT);
    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);

}
void loop() {

    if (digitalRead(SWTPIN)){
        Robawt.setSteer(30, 0);
    } else {
        Robawt.setSteer(0, 0);
        Robawt.reset();
    }

}