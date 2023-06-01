#include <Arduino.h>
#include <Wire.h>
#include "Vroom.h"
#include "VL53L1X.h"
#include "VL53L0X.h"

#define SDAPIN 4
#define SCLPIN 5
#define TCAADDR 0x70

#define l0x 1
#define l1x 1

#define L_LIDAR 6
#define R_LIDAR 0
#define F_LIDAR 1
#define FL_LIDAR 5
#define FR_LIDAR 2
#define FB_LIDAR 4

VL53L0X l_lidar, r_lidar, f_lidar, fl_lidar, fr_lidar;
VL53L1X fb_lidar;

int l_reading, r_reading,f_reading, fl_reading, fr_reading, fb_reading;

void setup(){
    Serial.begin(9600);
    while (!Serial) delay(10);

    Wire.setSDA(SDAPIN);
    Wire.setSCL(SCLPIN);
    Wire.begin();
    Wire.setClock(400000);
    
    #if l0x
    //^ use if lazy
    // for (int i = 0; i<8; i++) 
    // {
    //     tcaselect(i);
    //     sensor.setTimeout(500);
    //     if (sensor.init()) {
    //         sensor.startContinuous();
    //         a = i;
    //         // Serial.print("Sensor at pin "); Serial.print(i); Serial.println("L0X failed to initialise");
    //     }
    // }
    // tcaselect(0);
    // r_lidar.setTimeout(500);
    // while (!r_lidar.init()) Serial.println("FRONT LIDAR FAILED TO INIT");
    // r_lidar.startContinuous();

    // tcaselect(6);
    // l_lidar.setTimeout(500);
    // while (!l_lidar.init()) Serial.println("LEFT LIDAR FAILED TO INIT");
    // l_lidar.startContinuous();
    initL0X(&l_lidar, L_LIDAR);
    initL0X(&r_lidar, R_LIDAR);
    initL0X(&f_lidar, 1);
    initL0X(&fl_lidar, 5);
    initL0X(&fr_lidar, 2);
    #endif

    #if l1x
    initL1X(&fb_lidar, FB_LIDAR);
    // tcaselect(4);
    // fb_lidar.setTimeout(500);
    // while (!fb_lidar.init()) Serial.println("FRONT BOTTOM LIDAR FAILED TO INIT");
    // fb_lidar.setDistanceMode(VL53L1X::Medium);
    // fb_lidar.startContinuous(20);
    #endif

}

void loop(){
    #if l0x
    // for (int i = 0; i<8; i++) 
    // {
    //     tcaselect(i);
    //     sensor.setTimeout(500);
    //     if (sensor.init()) {
    //         sensor.startContinuous();
    //         a = i;
    //         // Serial.print("Sensor at pin ");
    //         // Serial.print(i);
    //         // Serial.println("L0X failed to initialise");
    //     }
    //     // sensor.startContinuous();
    // }

    // tcaselect(0);
    // if (r_lidar.available()){
    //     Serial.print("Right lidar: "); Serial.println(r_lidar.readRangeMillimeters());
    // }

    // tcaselect(6);
    // if (l_lidar.available()){
    //     Serial.print("Left lidar: ");Serial.println(l_lidar.readRangeMillimeters());
    // }
    //^ DOM: VV DUMB WAY TO CALL THE FUNCTION LOLLL I CAN'T THINK OF ANY BETTER WAYS SORRY i aspire to become a software architect
    if (readL0X(&l_lidar, L_LIDAR, l_reading)) {;}
    if (readL0X(&r_lidar, R_LIDAR, r_reading)) {;}
    if (readL0X(&f_lidar, F_LIDAR, f_reading)) {;}
    if (readL0X(&fl_lidar, FL_LIDAR, fl_reading)) {;}
    if (readL0X(&fr_lidar, FR_LIDAR, fr_reading)) {;}

    Serial.print("Front: "); Serial.print(f_reading); Serial.print(" | ");  
    Serial.print("Left: "); Serial.print(l_reading); Serial.print(" | ");
    Serial.print("Right: "); Serial.print(r_reading); Serial.print(" | ");
    Serial.print("Front left: "); Serial.print(fl_reading); Serial.print(" | ");
    Serial.print("Front right: "); Serial.print(fr_reading); Serial.print(" | ");    
    #endif

    #if l1x
    // tcaselect(4);
    // if (fb_lidar.dataReady()){
    //     fb_reading = fb_lidar.read(false);
    // }
    if (readL1X(&fb_lidar, FB_LIDAR, fb_reading)) {;}
    Serial.print("Front bottom: "); Serial.print(fb_reading); Serial.print(" | "); 
    #endif
    Serial.println();
}

void tcaselect(uint8_t i)  //I2C Multiplexer: TCA9548A
{
    if (i >= 0 && i <= 7) {
        Wire.beginTransmission(TCAADDR);
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

bool readL0X(VL53L0X *lidar, int pin, int& reading){
    tcaselect(pin);
    if(lidar->available()) {
        reading = lidar->readRangeMillimeters();
        return true;
    }
    return false;
}

bool readL1X(VL53L1X *lidar, int pin, int& reading){
    tcaselect(pin);
    if(lidar->dataReady()) {
        reading = lidar->read(false);
        return true;
    }
    return false;
}