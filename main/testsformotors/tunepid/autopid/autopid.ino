#include <PID_v1.h>
#include <Arduino.h>
#include <PID_AutoTune_v0.h>

double inputRpm, outputRpm, setpointRpm = 30;
double kp = 0.5, ki = 9, kd = 0;
int pin1 = 12;
int pin2 = 13;
int encPinA = 0;
int encPinB = 1;
int _encDir = 1;
long _begin, _end;
long _encVal;


byte ATuneModeRemember=2;
unsigned long serialTime, modelTime;
double aTuneStep=50, aTuneNoise=1, aTuneStartValue=100;
unsigned int aTuneLookBack=20;
bool tuning = false;

PID motorPID(&inputRpm, &outputRpm, &setpointRpm, kp, ki, kd, DIRECT);
PID_ATune aTune(&inputRpm, &outputRpm);

void setup() {
    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(encPinA, INPUT_PULLDOWN);
    pinMode(encPinB, INPUT_PULLDOWN);
    motorPID.SetMode(AUTOMATIC);

    attachInterrupt(encPinA, readEncA, RISING);
    attachInterrupt(encPinB, readEncB, RISING);
    
    if(tuning)
    {
        tuning=false;
        changeAutoTune();
        tuning=true;
    }
    
    serialTime = 0;

    Serial.begin(115200);
    while (!Serial) { delay(10); }
    Serial.println("Serial initialised!");
}

void loop() {
    inputRpm = getRpm();

    if(tuning)
    {
        byte val = (aTune.Runtime());
        if (val!=0)
        {
        tuning = false;
        }
        if(!tuning)
        { //we're done, set the tuning parameters
        kp = aTune.GetKp();
        ki = aTune.GetKi();
        kd = aTune.GetKd();
        motorPID.SetTunings(kp,ki,kd);
        AutoTuneHelper(false);
        }
    }
    else motorPID.Compute();

    setRpm();
    
    if(millis()>serialTime)
    {
        SerialReceive();
        SerialSend();
        serialTime+=500;
    }
}

void readEncA() 
{
    if(!digitalRead(encPinB)) {
        _begin = _end;
        _end = micros();
        _encVal ++;
        _encDir = 1; //forward
    }
}

void readEncB() 
{
    if (!digitalRead(encPinA)) {
        _begin = _end;
        _end = micros();
        _encVal --; 
        _encDir = -1; //backward
    }
}

double getRpm() 
{
    if (_begin && _end) {
        long lastInterval = _end - _begin;
        long nowInterval = micros()-_end; //1/370*60 
        if (lastInterval < nowInterval)
            return (double)((162162/nowInterval)*_encDir);
        else
            return (double)((162162/lastInterval)*_encDir);
    } else {
        return 0;
    }
}

void setRpm() //* rev min^-1
{
    if (outputRpm >= 0) {
        digitalWrite(pin1, LOW);
        analogWrite(pin2, (int)(outputRpm)); 
    } else {
        analogWrite(pin1, (int)(outputRpm));
        digitalWrite(pin2, LOW);
    }
}


void SerialSend()
{
  Serial.print("setpoint: ");Serial.print(setpointRpm); Serial.print(" ");
  Serial.print("input: ");Serial.print(inputRpm); Serial.print(" ");
  Serial.print("output: ");Serial.print(outputRpm); Serial.print(" ");
  if(tuning){
    Serial.println("tuning mode");
  } else {
    Serial.print("kp: ");Serial.print(motorPID.GetKp());Serial.print(" ");
    Serial.print("ki: ");Serial.print(motorPID.GetKi());Serial.print(" ");
    Serial.print("kd: ");Serial.print(motorPID.GetKd());Serial.println();
  }
}

void SerialReceive()
{
  if(Serial.available())
  {
   char b = Serial.read(); 
   Serial.flush(); 
   if((b=='1' && !tuning) || (b!='1' && tuning))changeAutoTune();
  }
}


void AutoTuneHelper(boolean start)
{
  if(start)
    ATuneModeRemember = motorPID.GetMode();
  else
    motorPID.SetMode(ATuneModeRemember);
}

void changeAutoTune()
{
 if(!tuning)
  {
    //Set the output to the desired starting frequency.
    outputRpm=aTuneStartValue;
    aTune.SetNoiseBand(aTuneNoise);
    aTune.SetOutputStep(aTuneStep);
    aTune.SetLookbackSec((int)aTuneLookBack);
    AutoTuneHelper(true);
    tuning = true;
  }
  else
  { //cancel autotune
    aTune.Cancel();
    tuning = false;
    AutoTuneHelper(false);
  }
}
