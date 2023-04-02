//* -------------------------------- PREPROCESSER DIRECTIVES --------------------------------
#include <Arduino.h>
#include <Wire.h>
#include "VL53L1X.h"
#include "VL53L0X.h"
#include "Vroom.h"
#include <Servo.h>

//* DEBUG SETTINGS
//^ Runs with code
#define debug_serial 0
#define debug_led 0
#define debug_looptime 0
#define debug_lidars 1

//* ADDRESSES
#define TCAADDR 0x70

//* CONSTANT PINS
#define TNYPIN1 9
#define TNYPIN2 8
#define TX0PIN 16
#define RX0PIN 17
#define SDAPIN 4
#define SCLPIN 5
#define SWTPIN 28
#define LEDPIN 3
#define ONBOARDLEDPIN 25

//* -------------------------------- START CODE --------------------------------

//* OBJECT INITIALISATIONS
Motor MotorR(13, 12, 19, 18); //M2 swapped
Motor MotorL(10, 11, 1, 0); //M1
Vroom Robawt(&MotorL, &MotorR);
VL53L0X lidarsl0x[4];
VL53L1X lidarsl1x[1];

//* MOTOR ENCODERS
void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }

//* LIDARS SETUP
//^ VL53L0X
int lidarl0x_readings[4] = {0, 0, 0, 0};
const int lidarl0x_pins[4] = {4, 3, 1, 5};
String lidarl0x_labels[4] = {"FRONT: ", "FRONT LEFT: ", "LEFT: ", "RIGHT: "};
namespace LidarsL0X {
    enum LidarsL0X { FRONT, FRONT_LEFT, LEFT, RIGHT };
}
//^ VL53L1X
int lidarl1x_readings[1] = {0};
const int lidarl1x_pins[1] = {2};
String lidarl1x_labels[1] = {"FRONT BOTTOM: "};
namespace LidarsL1X {
    enum LidarsL1X { FRONT_BOTTOM };
}
//^ Debugging Lidars
const int l0x_start = LidarsL0X::FRONT, //first lidar
    l0x_stop = LidarsL0X::RIGHT; //last l0x lidar
const int l1x_start = LidarsL1X::FRONT_BOTTOM, //first l1x lidar
    l1x_stop = LidarsL1X::FRONT_BOTTOM; //last l1x lidar

//* VARIABLES

//^ Debug
long firstLoopTimeMicros,
    beforeLidarLoopTimeMicros,
    afterLidarLoopTimeMicros;
bool led_on = false;

//^ 135
bool left135 = false,
    right135 = false;
long last135Millis, start135Millis;

//^ 90
long start90millis;

//^ Movement and logic
double rotation = 0,
    rpm = 40;
int serialState = 0,
    task = 0, 
    prev_task = 0,
    curr = 0;

//^ Evac
bool in_evac = false;

//* -------------------------------- START SETUP --------------------------------

void setup() 
{
    Serial.begin(9600);
    while (!Serial) delay(10); 
    Serial.println("USB serial initialised");

    pinMode(SWTPIN, INPUT_PULLDOWN);
    pinMode(ONBOARDLEDPIN, OUTPUT);
    pinMode(LEDPIN, OUTPUT);
    pinMode(TNYPIN1, INPUT);
    pinMode(TNYPIN2, INPUT);

    //^ PI SERIAL COMMS
    Serial1.setRX(RX0PIN);
    Serial1.setTX(TX0PIN);
    Serial1.begin(9600); //consider increasing baud
    while (!Serial1) delay(10); 
    Serial.println("Pi serial initialised");

    //^ MULTIPLEXER
    Wire.setSDA(SDAPIN);
    Wire.setSCL(SCLPIN);
    Wire.begin();
    Wire.setClock(400000); 

    //^ LIDAR INITIALISATIONS
    for (int i = l0x_start; i != (l0x_stop+1); i++) 
    {
        Serial.println(i);
        tcaselect(lidarl0x_pins[i]);
        lidarsl0x[i].setTimeout(500);
        while (!lidarsl0x[i].init()) {
            Serial.print(lidarl0x_labels[i]);
            Serial.print("at pin ");
            Serial.print(lidarl0x_pins[i]);
            Serial.print(" - ");
            Serial.println("L0X failed to initialise");
        }
        lidarsl0x[i].startContinuous();
    }
    for (int i = l1x_start; i != (l1x_stop+1); i++) 
    {
        Serial.println(i);
        tcaselect(lidarl1x_pins[i]);
        lidarsl1x[i].setTimeout(500);
        while (!lidarsl1x[i].init()) {
            Serial.print(lidarl1x_labels[i]);
            Serial.print("at pin ");
            Serial.print(lidarl1x_pins[i]);
            Serial.print(" - ");
            Serial.println("L1X failed to initialise");
        }
        lidarsl1x[i].setDistanceMode(VL53L1X::Medium);
        lidarsl1x[i].startContinuous(20);
    }

    //^ MOTOR ENCODERS
    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

//* -------------------------------- START LOOP --------------------------------

void loop() {

    //  teensyEvent();
    serialEvent();

    //* LIDAR READINGS
    #if debug_looptime
    beforeLidarLoopTimeMicros = micros();
    #endif
    for (int i = l0x_start; i != (l0x_stop+1); i++) 
    {
        tcaselect(lidarl0x_pins[i]);
        if(lidarsl0x[i].available()) {
            lidarl0x_readings[i] = lidarsl0x[i].readRangeMillimeters();
        }
    }
    for (int i = l1x_start; i != (l1x_stop+1); i++) 
    {
        tcaselect(lidarl1x_pins[i]);
        if(lidarsl1x[i].dataReady()) {
            lidarl1x_readings[i] = lidarsl1x[i].read(false);
        }
    }
    #if debug_looptime
    afterLidarLoopTimeMicros = micros();
    #endif

    //* SWITCH ON
    if (!digitalRead(SWTPIN))
    {

        //* TASK FROM PI
        switch (task) 
        {
            case 0: //~ empty linetrack
                if (curr == 5 || curr == 6) {
                if (millis() - start90millis > 500) { curr = 0; }
                } else {
                    curr = 0;
                }
                break;
            
            case 4: //~ red line --> stop
                curr = 4;
                break;  

            case 5: //~ left 90
                if (curr == 0) { start90millis = millis(); }
                if (curr == 6) { break; }
                curr = 5;
                break;  

            case 6: //~ right 90
                if (curr == 0) { start90millis = millis(); }
                if (curr == 5) { break; }
                curr = 6;
                break;  
        }   

        //* CURRENT ACTION HANDLED
        switch (curr)
        {
            case 0: //~ empty linetrack
                Robawt.setSteer(rpm, rotation);
                break;

            case 4: //~ red --> stop
                Robawt.setSteer(0, 0);
                break;

            case 5: //~ TURN LEFT 90
                Robawt.setSteer(rpm, -1);
                #if debug_led
                led_on = true;
                #endif
                break;

            case 6: //~ TURN RIGHT 90
                Robawt.setSteer(rpm, 1);
                #if debug_led
                led_on = true;
                #endif
                break;
        }

    //* SWITCH OFF
    } else {
        Robawt.setSteer(0, 0);
        Robawt.reset();
    }

    //* DEBUG PRINTS
    
    #if debug_lidars
    for (int i = l0x_start; i != (l0x_stop+1); i++) {
        Serial.print(lidarl0x_labels[i]);
        Serial.print(lidarl0x_readings[i]);
        // if (lidarsl0x[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
        Serial.print(" || ");
    }
    for (int i = l1x_start; i != (l1x_stop+1); i++) {
        Serial.print(lidarl1x_labels[i]);
        Serial.print(lidarl1x_readings[i]);
        // if (lidarsl1x[i].timeoutOccurred()) { Serial.print(" TIMEOUT"); }
        Serial.print(" || ");
    }
    Serial.println();
    #endif

    #if debug_led
    if (led_on) { digitalWrite(ONBOARDLEDPIN, HIGH); }
    else { digitalWrite(ONBOARDLEDPIN, LOW); }
    led_on = false;
    #endif

    #if debug_looptime
    Serial.print("Time loop 1 (micros): ");
    Serial.print(micros() - firstLoopTimeMicros);
    firstLoopTimeMicros = micros();
    Serial.print(" || ");
    Serial.print("Lidar reading time (micros): ");
    Serial.print(afterLidarLoopTimeMicros-beforeLidarLoopTimeMicros);
    Serial.println();
    #endif
}

//* -------------------------------- USER DEFINED FUNCTIONS --------------------------------

void teensyEvent()
{
    int pin1State = digitalRead(TNYPIN1);
    int pin2State = digitalRead(TNYPIN2);
    Serial.print("pin1state: " );
    Serial.println(pin1State);
    Serial.print("pin2state: " );
    Serial.println(pin2State);
    if (pin1State && pin2State) { //double black 11
        curr = 0;
    } else if (!in_evac && !pin1State && pin2State) { //left 01
        curr = 1;
    } else if (!in_evac && pin1State && !pin2State) { //right 10
        curr = 2;
    } else {
        curr = 3;
    }
}

void serialEvent()
{
  while (Serial1.available()) 
  {
    int serialData = Serial1.read();
    if (serialData == 255 || serialData == 254 || serialData == 253 || serialData == 252) {
      serialState = (int)serialData;
      #if debug_serial
      Serial.print("Serial State: ");
      Serial.println(serialState);
      #endif
    } else {
      switch (serialState) 
      {
        case 255:
          rotation = ((double)(serialData)-90)/90;
          break;
        case 254:
          rpm = (double)serialData;
          break;
        case 253:
          task = (int)serialData;
          break;
      }
      #if debug_serial
      Serial.print("Data: ");
      Serial.println(serialData);
      #endif
    }
  }
}


void tcaselect(uint8_t i)  //Multiplexer: TCA9548A
{
  if (i > 0 && i < 7) {
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << i);
    Wire.endTransmission();
  }
}
