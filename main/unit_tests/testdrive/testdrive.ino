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


Motor MotorL(12, 13, 0, 1); 
Motor MotorR(11, 10, 19, 18);
Vroom Robawt(&MotorL, &MotorR);

void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }

unsigned long long serialTime;
double rpm = 0;

void setup(){
    Serial.begin(115200);
    pinMode(SWTPIN, INPUT);
    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
    serialTime = millis();
}
void loop() {
    
    if (digitalRead(SWTPIN)){
        
        // for (int i=0; i<100; i=i+5){
        //     Robawt.setSteer(i, 0);
        //     Serial.println(i);
        //     delay(1000);
        // }
        // Robawt.setSteer(80, 1);
        MotorL.setVal(-80);
        Serial.println("running?");

    } else {
        MotorL.setVal(0);
    }

    // if(millis()>serialTime)
    // {
    //     SerialReceive();
    //     serialTime+=500;
    // }

}

void SerialReceive()
{
  if(Serial.available())
  {
   int b = Serial.read(); 
   Serial.flush(); 
   Serial.println(b);
  }
}