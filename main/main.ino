// --------------------------------------
// main.cpp
//
// Main script on robot
// --------------------------------------

#include <Arduino.h>
#include <Wire.h>
#include "VL53L1X.h"
#include "VL53L0X.h"
#include "Vroom.h"
// #include "Lidar.h"
// #include "Claw.h"

#define TCAADDR 0x70

//* OBJECT INITIALISATIONS */
Motor MotorL(13, 12, 19, 18); //M2 swapped
Motor MotorR(10, 11, 16, 17); //M1
Vroom Robawt(&MotorL, &MotorR);
VL53L0X sensor;

//* MOTOR ENCODERS */
void ISRLA() {MotorL.readEncA();}
void ISRLB() {MotorL.readEncB();}
void ISRRA() {MotorR.readEncA();}
void ISRRB() {MotorR.readEncB();}

//* DEPRECATED CODE */
// MUX Mux(&Wire, 20, 21);
// L1X LidarFL(&Mux, 4);
// L0X LidarClaw(6);

//* CONSTANTS
const int TX1PIN = 8,
  RX1PIN = 9,
  SDAPIN = 20,
  SCLPIN = 21,
  SWTPIN = 14;

int serialState = 0;

double rotation = 0;
double rpm = 40;
int task = 0;

void setup() {

  pinMode(SWTPIN, INPUT);

  //* USB SERIAL COMMS */
  Serial.begin(9600);
  while (!Serial)
     delay(10);
  Serial.println("USB serial initialised");

  //* PI SERIAL COMMS */
  // Serial2.setRX(RX1PIN);
  // Serial2.setTX(TX1PIN);
  // Serial2.begin(9600); //consider increasing baud
  // while (!Serial2) delay(10);
  // Serial.println("Pi serial initialised");

  //* MULTIPLEXER */
  Wire.setSDA(SDAPIN);
  Wire.setSCL(SCLPIN);
  Wire.begin();
  Wire.setClock(400000);

  // tcaselect(3);
  // sensor.setTimeout(500);
  // while (!sensor.init()) {Serial.println("L0X failed to initialise");}
  // sensor.startContinuous(50);

  // businit(SDAPIN, SCLPIN);
  // l0xinit(&l_tof, 3);
  // l0xinit(r_tof, 5);

  //* MOTOR ENCODERS */
  attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
  attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
  attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
  attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

// L1X LidarFR(4);


void loop() {

  serialEvent();

  if (digitalRead(SWTPIN)) {

    //* ACTUAL CODE
    // switch (task) {

    //   case 0: //normal lt
    //     Robawt.setSteer(rpm, rotation);
    //     break;

    //   case 1: //left gs
    //     Robawt.setSteer(rpm, -0.5);
    //     break;
      
    //   case 2: //right gs
    //     Robawt.setSteer(rpm, 0.5);
    //     break;

    //   case 3: //double gs
    //     Robawt.setSteer(50, 1);
    //     break;

    //   case 4: //red line
    //     Robawt.setSteer(0, 0);
    //     Robawt.reset();
    //     break;


    //   case 5: //moving backwards (blue)
    //     Robawt.setSteer(-rpm, 0);
    //     break;

    // }

    //* DEBUG LIDARS (DEBUG CODE NOT WORKING LOL) */
    // int value = l0xread(&, 3);

    // tcaselect(3);
    // int value = sensor.readRangeContinuousMillimeters();
    // if (sensor.timeoutOccurred()) Serial.println("TIMEOUT");
    // else Serial.println(value);

    //* DEBUG PASSED VARIABLES */
    // Serial.print("task: ");
    // Serial.println(task);
    // Serial.print("rotation: ");
    // Serial.println(rotation);
    // Serial.print("rpm: ");
    // Serial.println(rpm);

    //* TEST PID */
    // double val = MotorL.setRpm(20);
    // Serial.print("Actual rpm: ");
    // Serial.print(MotorL.getRpm());
    // Serial.print(" Output pwm: ");
    // Serial.println(val);

  } else {

    // startChangeMillis = millis();

    //* TO MAKE ROBOT STOP */
    Robawt.setSteer(0, 0);
    Robawt.reset();

    //* TO MAKE LEFT MOTOR STOP */
    // MotorL.setRpm(0);
    // MotorL.resetPID();
  }
}


//* FUNCTIONS */

void serialEvent()
{
  while (Serial2.available()) {
    int serialData = Serial2.read();
    if (serialData == 255 || serialData == 254 || serialData == 253) {serialState = (int)serialData;}
    switch (serialState) {
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
  }
}

void tcaselect(uint8_t i)
{
  if (i < 0 || i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();
}

void tcaselect1(uint8_t i)
{
  if (i < 0 || i > 7) return;
  Wire1.beginTransmission(TCAADDR);
  Wire1.write(1 << i);
  Wire1.endTransmission();
}



// void l0xinit(VL53L0X *sensor, uint8_t pin)
// {
//   tcaselect(pin);
//   sensor->setTimeout(500);
//   while (!sensor->init()) {Serial.println("L0X failed to initialise");}
//   sensor->startContinuous(50);
// }

// int l0xread(VL53L0X *sensor, uint8_t pin)
// {
//   tcaselect(pin);
//   int value = sensor->readRangeContinuousMillimeters();
//   if (sensor->timeoutOccurred()) return -1;
//   else return value;
// }

// void l1xinit(VL53L1X *sensor, uint8_t pin)
// {
//   tcaselect(pin);
//   sensor.setTimeout(500);
//   while (!sensor.init()) {Serial.println("L1X failed to initialise");}
//   sensor.setDistanceMode(VL53L1X::Medium);
//   sensor.startContinuous(50);
// }

// int l1xread(VL53L1X *sensor, uint8_t pin)
// {
//   tcaselect(pin);
//   int value = sensor.read();
//   if (sensor.timeoutOccurred()) return -1;
//   else return value;
// }

// void businit(int sdaPin, int sclPin)
// {
//   Wire1.setSDA(sdaPin);
//   Wire1.setSCL(sclPin);
//   Wire1.begin();
//   Wire1.setClock(400000); 
// }