#include <pidautotuner.h>
#include <PID_v1.h>

double inputRpmL, outputRpmL, setpointRpmL = 30;
double inputRpmR, outputRpmR, setpointRpmR = 30;
double kp_r = 0.5, ki_r = 9, kd_r = 0;
double kp_l = 0.5, ki_l = 9, kd_l = 0;
int pinL1 = 12, pinL2 = 13;
int pinR1 = 11, pinR2 = 10;
int encPinLA = 0, encPinLB = 1;
int encPinRA = 19, encPinRB = 18;
int _encDir = 1;
long _beginL, _endL;
long _beginR, _endR;
long _encValL, _encValR;


PID motorPIDL(&inputRpmL, &outputRpmL, &setpointRpmL, 4.67, 0.62, 9.88, DIRECT);
PID motorPIDR(&inputRpmR, &outputRpmR, &setpointRpmR, 4.67, 0.62, 9.88, DIRECT);

//  4.67
// ki: 0.62
// kd: 9.88


void setup() {

    pinMode(pin1L, OUTPUT);
    pinMode(pin2L, OUTPUT);
    pinMode(pin1R, OUTPUT);
    pinMode(pin2R, OUTPUT);
    pinMode(28, INPUT_PULLDOWN);
    pinMode(encPinLA, INPUT_PULLDOWN);
    pinMode(encPinLB, INPUT_PULLDOWN);
    pinMode(encPinRA, INPUT_PULLDOWN);
    pinMode(encPinRB, INPUT_PULLDOWN);
    motorPID.SetMode(AUTOMATIC);

    attachInterrupt(encPinLA, readEncLA, RISING);
    attachInterrupt(encPinLB, readEncLB, RISING);
    attachInterrupt(encPinRA, readEncRA, RISING);
    attachInterrupt(encPinRB, readEncRB, RISING);


    Serial.begin(115200);
    while (!digitalRead(28)) { delay(10); }

    PIDAutotuner tunerL = PIDAutotuner();
    PIDAutotuner tunerR = PIDAutotuner();

    // Set the target value to tune to
    // This will depend on what you are tuning. This should be set to a value within
    // the usual range of the setpoint. For low-inertia systems, values at the lower
    // end of this range usually give better results. For anything else, start with a
    // value at the middle of the range.
    tunerL.setTargetInputValue(30);
    tunerR.setTargetInputValue(30);

    // Set the loop interval in microseconds
    // This must be the same as the interval the PID control loop will run at
    tunerL.setLoopInterval(100000);
    tunerR.setLoopInterval(100000);

    // Set the output range
    // These are the minimum and maximum possible output values of whatever you are
    // using to control the system (Arduino analogWrite, for example, is 0-255)
    tunerL.setOutputRange(0, 255);
    tunerR.setOutputRange(0, 255);

    // Set the Ziegler-Nichols tuning mode
    // Set it to either PIDAutotuner::ZNModeBasicPID, PIDAutotuner::ZNModeLessOvershoot,
    // or PIDAutotuner::ZNModeNoOvershoot. Defaults to ZNModeNoOvershoot as it is the
    // safest option.
    tunerL.setZNMode(PIDAutotuner::ZNModeBasicPID);
    tunerR.setZNMode(PIDAutotuner::ZNModeBasicPID);

    // This must be called immediately before the tuning loop
    // Must be called with the current time in microseconds
    tunerL.startTuningLoop(micros());
    tunerL.setTuningCycles(50);
    tunerR.startTuningLoop(micros());
    tunerR.setTuningCycles(50);

    // Run a loop until tunerL.isFinished() returns true
    long microseconds;
    while (!tunerL.isFinished() || !tunerR.isFinished()) {

        // This loop must run at the same speed as the PID control loop being tuned
        long prevMicroseconds = microseconds;
        microseconds = micros();

        // Get input value here (temperature, encoder position, velocity, etc)
        double inputL = getRpmL();
        double inputR = getRpmR();

        // Call tunePID() with the input value and current time in microseconds
        double outputL = tunerL.tunePID(input, microseconds);
        double outputR = tunerL.tunePID(input, microseconds);

        // Set the output - tunePid() will return values within the range configured
        // by setOutputRange(). Don't change the value or the tuning results will be
        // incorrect.
        setRpmL(outputL);
        setRpmR(outputR);

        // This loop must run at the same speed as the PID control loop being tuned
        while (micros() - microseconds < 100000) delayMicroseconds(1);
    }

    // Turn the output off here.
    setRpmL(0);
    setRpmR(0);

    // Get PID gains - set your PID controller's gains to these
    kp_l = tunerL.getKp();
    ki_l = tunerL.getKi();
    kd_l = tunerL.getKd();
    Serial.print("kp: "); Serial.println(kp_l);
    Serial.print("ki: "); Serial.println(ki_l);
    Serial.print("kd: "); Serial.println(kd_l);
    motorPIDL.SetTunings(kp_l,ki_l,kd_l);

    kp_r = tunerR.getKp();
    ki_r = tunerR.getKi();
    kd_r = tunerR.getKd();
    Serial.print("kp: "); Serial.println(kp_r);
    Serial.print("ki: "); Serial.println(ki_r);
    Serial.print("kd: "); Serial.println(kd_r);
    motorPIDL.SetTunings(kp_r,ki_r,kd_r);
}

void loop() {
    if (digitalRead(28)) {
        inputRpmL = getRpm();
        motorPIDL.Compute();
        setRpmL(outputRpmL);
        inputRpmR = getRpm();
        motorPIDR.Compute();
        setRpmR(outputRpmR);
    } else {
        setRpmL(0);
        setRpmR(0);
        Serial.print("right kp: "); Serial.print(kp_r);
        Serial.print(" right ki: "); Serial.print(ki_r);
        Serial.print(" right kd: "); Serial.println(kd_r);
        Serial.print("left kp: "); Serial.print(kp_l);
        Serial.print(" left ki: "); Serial.print(ki_l);
        Serial.print(" left kd: "); Serial.println(kd_l);
    }
}

void readEncLA() 
{
    if(!digitalRead(encPinLB)) {
        _beginL = _endL;
        _endL = micros();
        _encValL ++;
        _encDirL = 1; //forward
    }
}

void readEncLB() 
{
    if (!digitalRead(encPinLA)) {
        _beginL = _endL;
        _endL = micros();
        _encValL --; 
        _encDirL = -1; //backward
    }
}

void readEncRA() 
{
    if(!digitalRead(encPinRB)) {
        _beginR = _endR;
        _endR = micros();
        _encValR ++;
        _encDirR = 1; //forward
    }
}

void readEncRB() 
{
    if (!digitalRead(encPinRA)) {
        _beginR = _endR;
        _endR = micros();
        _encValR --; 
        _encDirR = -1; //backward
    }
}


double getRpmL() 
{
    if (_beginL && _endL) {
        long lastInterval = _endL - _beginL;
        long nowInterval = micros()-_end; //1/370*60 
        if (lastInterval < nowInterval)
            return (double)((162162/nowInterval)*_encDirL);
        else
            return (double)((162162/lastInterval)*_encDirL);
    } else {
        return 0;
    }
}

double getRpmR() 
{
    if (_beginR && _endR) {
        long lastInterval = _endR - _beginR;
        long nowInterval = micros()-_end; //1/370*60 
        if (lastInterval < nowInterval)
            return (double)((162162/nowInterval)*_encDirR);
        else
            return (double)((162162/lastInterval)*_encDirR);
    } else {
        return 0;
    }
}


void setRpmL(double output) //* rev min^-1
{
    if (output >= 0) {
        digitalWrite(pinL1, LOW);
        analogWrite(pinL2, (int)(output)); 
    } else {
        analogWrite(pinL1, (int)(output));
        digitalWrite(pinL2, LOW);
    }
}


void setRpmR(double output) //* rev min^-1
{
    if (output >= 0) {
        digitalWrite(pinR1, LOW);
        analogWrite(pinR2, (int)(output)); 
    } else {
        analogWrite(pinR1, (int)(output));
        digitalWrite(pinR2, LOW);
    }
}