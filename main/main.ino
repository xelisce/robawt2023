// --------------------------------------
// main.cpp
//
// Main script on robot
// --------------------------------------

#include <Wire.h>
#include <VL53L1X.h>
#include <Vroom.h>

VL53L1X sensor;

Motor MotorL(13, 12, 19, 18); //M2 swapped
Motor MotorR(10, 11, 16, 17); //M1
Vroom Robawt(&MotorL, &MotorR);

void ISRLA() {MotorL.readEncA();}
void ISRLB() {MotorL.readEncB();}
void ISRRA() {MotorR.readEncA();}
void ISRRB() {MotorR.readEncB();}

#define WIRE Wire
#define TCAADDR 0x70

#define TX1 8
#define RX1 9
#define SWTPIN 14
#define SDA0PIN 20
#define SCL0PIN 21

int rotation = 0;
int caseSwitch = 0;
volatile long temp, counter = 0;
long prevSwitchMil;

void serialEvent() 
{
  while (Serial2.available()) {
    rotation = (Serial2.read() - 90)/90;
  }
}



// 10=0, 11=+, Right motor backwards
// 12=0, 13=+, left forwards
void setup() {
  /* MULTIPLEXER I2C */
  // Wire.setSCL(SCL0PIN);
  // Wire.setSDA(SDA0PIN);
  // Wire.begin();
  // Wire.setClock(400000);

  /* USB SERIAL COMMS */
  // Serial.begin(9600);
  // while (!Serial)
  //    delay(10);
  // Serial.println("USB serial initialised");

  /* PI SERIAL COMMS */
  // Serial2.setRX(RX1);
  // Serial2.setTX(TX1);
  // Serial2.begin(9600); //consider increasing baud
  // while (!Serial2)
  //   delay(10);
  // Serial.println("Pi serial initialised");

  pinMode(SWTPIN, INPUT);

  prevSwitchMil = millis();

  attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING); //double check if its change or rising
  attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
  attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
  attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}


void loop() {

  // read_sensor(4);

  // serialEvent();

  // if (counter != temp)
  // {
  //   Serial.println(counter);
  //   temp = counter;
  // }



  if (digitalRead(SWTPIN)) 
  {
    // switch (caseSwitch)
    // {
    //   case 0:
    //     if (millis()-prevSwitchMil < 2000) {
    //       Robawt.setSteer(0.5, 0);
    //     } else {
    //       prevSwitchMil = millis();
    //       caseSwitch = 1;
    //     }
    //     break;

    //   case 1:
    //     if (millis()-prevSwitchMil < 2000) {
    //       Robawt.setSteer(0.5, 1);
    //     } else {
    //       prevSwitchMil = millis();
    //       caseSwitch = 2;
    //     }
    //     break;
      
    //   case 2:
    //     if (millis()-prevSwitchMil < 2000) {
    //       Robawt.setSteer(0.5, -1);
    //     } else {
    //       prevSwitchMil = millis();
    //       caseSwitch = 3;
    //     }
    //     break;

    //   case 3:
    //     if (millis()-prevSwitchMil < 2000) {
    //       Robawt.setSteer(-0.5, 1);
    //     } else {
    //       prevSwitchMil = millis();
    //       caseSwitch = 4;
    //     }
    //     break;
      
    //   case 4:
    //     if (millis()-prevSwitchMil < 2000) {
    //       Robawt.setSteer(-0.5, -1);
    //     } else {
    //       prevSwitchMil = millis();
    //       caseSwitch = 5;
    //     }
    //     break;

    //   case 5:
    //     if (millis()-prevSwitchMil < 2000) {
    //       Robawt.setSteer(-0.5, 0);
    //     } else {
    //       prevSwitchMil = millis();
    //       caseSwitch = 0;
    //     }
    //     break;

    // }

    
  } else {
    Robawt.setSteer(0, 0);
  }



}





void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void init_sensor(int i) {
  tcaselect(i);
  sensor.setTimeout(500);
  if (!sensor.init())
  {
    Serial.println("Failed to detect and initialize sensor!");
    while (1);
  }
  sensor.setDistanceMode(VL53L1X::Medium);
  sensor.setMeasurementTimingBudget(50000);
  sensor.startContinuous(50);
}

void read_sensor(int i) {
  tcaselect(i);
  Serial.print(sensor.read());
  if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
}



