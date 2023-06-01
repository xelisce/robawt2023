#include <Arduino.h>
#include <Vroom.h>
#include "VL53L1X.h"
#include "VL53L0X.h"

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

VL53L0X r_lidar, l_lidar, f_lidar, fl_lidar, fr_lidar;
VL53L1X fb_lidar;

int f_reading, l_reading, r_reading, fb_reading;
int curr = 0;

int obstState = 0;
int turn_dir;
double obstDist, obstStartTurningBackDist;
int* sideObstDist;
long obstTime;

void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }


void setup(){

    Serial.begin(9600);
    while (!Serial) delay(10);
    Serial.println("Serial initialised");
    Wire.setSDA(4);
    Wire.setSCL(5);
    Wire.begin();
    Wire.setClock(400000);

    initL0X(&l_lidar, 6);
    initL0X(&r_lidar, 0);
    initL0X(&f_lidar, 1);
    initL1X(&fb_lidar, 4);

    // tcaselect(0);
    // r_lidar.setTimeout(500);
    // while (!r_lidar.init()) Serial.println("RIGHT LIDAR FAILED TO INIT");
    // r_lidar.startContinuous();

    // tcaselect(6);
    // l_lidar.setTimeout(500);
    // while (!l_lidar.init()) Serial.println("LEFT LIDAR FAILED TO INIT");
    // l_lidar.startContinuous();

    // tcaselect(1);
    // f_lidar.setTimeout(500);
    // while (!f_lidar.init()) Serial.println("FRONT LIDAR FAILED TO INIT");
    // f_lidar.startContinuous();

    // tcaselect(4);
    // fb_lidar.setTimeout(500);
    // while (!fb_lidar.init()) {Serial.println("L1X at pin FAILED TO INIT");}
    // fb_lidar.setDistanceMode(VL53L1X::Medium);
    // fb_lidar.startContinuous(20);
    
    pinMode(SWTPIN, INPUT);
    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}
void loop() {

    tcaselect(4); if(fb_lidar.dataReady()) {fb_reading = fb_lidar.read(false);}
    tcaselect(1); if(f_lidar.available()) {f_reading = f_lidar.readRangeMillimeters();}
    tcaselect(6); if(l_lidar.available()) {l_reading = l_lidar.readRangeMillimeters();}
    tcaselect(0); if(r_lidar.available()) {r_reading = r_lidar.readRangeMillimeters();}

    Serial.print("Front bottom: "); Serial.print(fb_reading); Serial.print(" | ");
    Serial.print("Front: "); Serial.print(f_reading); Serial.print(" | ");  
    Serial.print("Left: "); Serial.print(l_reading); Serial.print(" | ");
    Serial.print("Right: "); Serial.print(r_reading); Serial.print(" | ");
    Serial.println();

    if (digitalRead(SWTPIN)){
        switch (curr) {
            case 0: //^ move forward until obst
                Robawt.setSteer(30, 0);
                if (fb_reading < 85 && f_reading < 40) {
                    curr = 1;
                    obstDist = MotorL.getDist();
                }
                break;

            case 1:
                Robawt.setSteer(-40, 0);
                // Serial.println(MotorL.getDist() - obstDist);
                if (fabs(MotorL.getDist() - obstDist) > 7.5) {
                    obstState = 0;
                    // sideObstDist = turn_dir == 1 ? &l0x_readings[L0X::LEFT] : &l0x_readings[L0X::RIGHT]; //^ getting address of readings to constantly update the val
                    //^ turn left first
                    sideObstDist = &r_reading;
                    turn_dir = -1;
                    curr = 2; 
                }
                break;

            case 2: //^ turning 90 degrees
                switch (obstState){
                    case 0:
                        Robawt.setSteer(40, turn_dir);
                        if (*sideObstDist > 300) obstState ++;
                        break;
                    case 1:
                        Robawt.setSteer(40, turn_dir);
                        if (*sideObstDist < 300) obstState ++;
                        break;
                    case 2:
                        Robawt.setSteer(40, 0);
                        obstState = 0;
                        obstTime = millis();
                        curr = 3;
                        break;
                }
                break;

            case 3: //^ going around obstacle
                switch (obstState) {
                    case 0:
                        Robawt.setSteer (40, 0);
                        if (*sideObstDist < 150) { obstState++; }
                        break;
                        
                    case 1:
                        Robawt.setSteer (40, 0);
                        if (*sideObstDist > 150) { obstState++; }
                        break;

                    case 2:
                        Robawt.setSteer (40, -0.5*turn_dir);
                        if (*sideObstDist < 150) { obstState++; }
                        break;

                    case 3:
                        Robawt.setSteer (40, -0.5*turn_dir);
                        if (*sideObstDist > 150) { obstState = 0; }
                        break;
                }
                if (millis() - obstTime > 6000){
                    obstState = 0;
                    obstStartTurningBackDist = MotorR.getDist();
                    curr = 4;
                }
                break;

            case 4: //^ turning back to line after obstacle
                Robawt.setSteer(30, turn_dir*0.7);
                if (fabs(MotorR.getDist() - obstStartTurningBackDist) > 25) {
                    curr = 0;
                }
                break;
        }
    } else {
        Robawt.setSteer(0, 0);
        Robawt.reset();
        curr = 0;
    }
}


void tcaselect(uint8_t i)  //I2C Multiplexer: TCA9548A
{
    if (i >= 0 && i <= 7) {
        Wire.beginTransmission(TCAADR);
        Wire.write(1 << i);
        Wire.endTransmission();
    }
}

void initL0X(VL53L0X* lidar, int pin){ 
    tcaselect(pin);
    lidar->setTimeout(500);
    while (!lidar->init()) {Serial.print("L0X at pin "); Serial.print(pin); Serial.println(" FAILED TO INIT");}
    lidar->startContinuous();
    Serial.println("Lidar Initialised");
}

void initL1X(VL53L1X *lidar, int pin){
    tcaselect(pin);
    lidar->setTimeout(500);
    while (!lidar->init()) {Serial.print("L1X at pin "); Serial.print(pin); Serial.println(" FAILED TO INIT");}
    lidar->setDistanceMode(VL53L1X::Medium);
    lidar->startContinuous(20);
    Serial.println("Lidar Initialised");
}