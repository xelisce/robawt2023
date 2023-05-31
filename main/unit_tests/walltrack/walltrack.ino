#include <Arduino.h>
#include <TCA9548A.h>
#include <VL53L0X.h>
#include <Vroom.h>

//* choose only one: left or right or both *//
#define left_track 1 //wall track using left diagonal lidar
#define right_track 0 //wall track using right diagonal lidar
#define both_track 0 //wall track based on switch is flipped or not
//* choose if print *//
#define debug_print 1 //requires usb to be plugged in for robot to move
//* choose constant dist or spiral *//
#define spiral 0
double setDist = 400;

#define TCAADR 0x70
#define L0XADR 0x29
#define L1XADR 0x29
#define TCSADR 0x29

#define LEDPIN 3
#define SDAPIN 4
#define SCLPIN 5
#define SDA1PIN 6
#define SCL1PIN 7
#define TX0PIN 16
#define RX0PIN 17
#define PICOLEDPIN 25
#define SWTPIN 28

#define LEFT_CH 5
#define RIGHT_CH 2

Motor MotorR(13, 12, 1, 0); 
Motor MotorL(10, 11, 18, 19);
Vroom Robawt(&MotorL, &MotorR);

TCA9548A TopMux(SDAPIN, SCLPIN, &Wire, 400000);

VL53L0X lidarleft, lidarright;

void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }

int dist_from_wall;
long evac_settime, startEvacMillis, scrap_wall_time;

double evac_setdist,
    evac_startdist,
    k_p_wall_rot,
    wall_rot;

void setup() {
    
    Serial.begin(9600);
    #if debug_print
    while (!Serial) delay(10);
    Serial.println("Serial initialised");
    #endif

    pinMode(SWTPIN, INPUT);
    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);

    TopMux.begin();

    #if left_track || both_track
    TopMux.select(LEFT_CH);
    lidarleft.setTimeout(500);
    while (!lidarleft.init()) Serial.println("LEFT LIDAR FAILED TO INIT");
    lidarleft.startContinuous();
    #if debug_print
    Serial.println("Left lidar initialised");
    #endif
    #endif

    #if right_track || both_track
    TopMux.select(RIGHT_CH);
    lidarright.setTimeout(500);
    while (!lidarright.init()) Serial.println("RIGHT LIDAR FAILED TO INIT");
    lidarright.startContinuous();
    #if debug_print
    Serial.println("Right lidar initialised");
    #endif
    #endif

    #if spiral
    startEvacMillis = millis();
    #endif
}

void loop() {
    if (digitalRead(SWTPIN)) 
    {
        #if left_track || both_track
        TopMux.select(LEFT_CH);
        if(lidarleft.available()) {
            dist_from_wall = lidarleft.readRangeMillimeters();
        }
        #endif
        #if right_track
        TopMux.select(RIGHT_CH);
        if(lidarright.available()) {
            dist_from_wall = lidarright.readRangeMillimeters();
        }
        #endif
        #if debug_print
        Serial.print("Distance from wall: ");
        Serial.println(dist_from_wall);
        #endif
        #if spiral
        evac_settime = millis() - (startEvacMillis + scrap_wall_time);
        evac_setdist = 100 + (evac_settime/200);
        if (evac_setdist > 600) {evac_setdist = 600;}
        k_p_wall_rot = 0.008;
        wall_rot = (evac_setdist - l0x_readings[L0X::FRONT_LEFT]) * k_p_wall_rot;
        Robawt.setSteer(evac_rpm, wall_rot);
        #if debug_print
        Serial.print("Set distance: ");
        Serial.println(evac_setdist);
        Serial.print("Rotation: ");
        Serial.println(wall_rot);
        #endif
        #endif
    } 
    else 
    {
        #if both_track
        TopMux.select(RIGHT_CH);
        if(lidarright.available()) {
            dist_from_wall = lidarright.readRangeMillimeters();
        }
        #if debug_print
        Serial.print("Distance from wall: ");
        Serial.println(dist_from_wall);
        #endif
        #if spiral
        evac_settime = millis() - (startEvacMillis + scrap_wall_time);
        evac_setdist = 100 + (evac_settime/200);
        if (evac_setdist > 600) {evac_setdist = 600;}
        k_p_wall_rot = 0.008;
        wall_rot = (evac_setdist - l0x_readings[L0X::FRONT_LEFT]) * k_p_wall_rot;
        Robawt.setSteer(evac_rpm, wall_rot);
        #if debug_print
        Serial.print("Set distance: ");
        Serial.println(evac_setdist);
        Serial.print("Rotation: ");
        Serial.println(wall_rot);
        #endif
        #endif
        #else
        Robawt.setSteer(0, 0);
        Robawt.reset();
        #endif
    }
}