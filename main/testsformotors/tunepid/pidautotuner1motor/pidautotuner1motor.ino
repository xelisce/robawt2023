#include <pidautotuner.h>
#include <PID_v1.h>
#include <Arduino.h>

double inputRpm, outputRpm, setpointRpm = 30;
double k_p = 0.5, k_i = 9, k_d = 0;
int pin1 = 12;
int pin2 = 13;
int encPinA = 0;
int encPinB = 1;
int _encDir = 1;
long _begin, _end;
long _encVal;


PID motorPID(&inputRpm, &outputRpm, &setpointRpm, 4.67, 0.62, 9.88, DIRECT);

//  4.67
// k_i: 0.62
// k_d: 9.88


void setup() {

    pinMode(pin1, OUTPUT);
    pinMode(pin2, OUTPUT);
    pinMode(28, INPUT_PULLDOWN);
    pinMode(encPinA, INPUT_PULLDOWN);
    pinMode(encPinB, INPUT_PULLDOWN);
    motorPID.SetMode(AUTOMATIC);
    motorPID.SetSampleTime(100);

    attachInterrupt(encPinA, readEncA, RISING);
    attachInterrupt(encPinB, readEncB, RISING);

    Serial.begin(115200);
    while (!digitalRead(28)) { delay(10); }

    PIDAutotuner tuner = PIDAutotuner();

    // Set the target value to tune to
    // This will depend on what you are tuning. This should be set to a value within
    // the usual range of the setpoint. For low-inertia systems, values at the lower
    // end of this range usually give better results. For anything else, start with a
    // value at the middle of the range.
    tuner.setTargetInputValue(30);

    // Set the loop interval in microseconds
    // This must be the same as the interval the PID control loop will run at
    tuner.setLoopInterval(100000);

    // Set the output range
    // These are the minimum and maximum possible output values of whatever you are
    // using to control the system (Arduino analogWrite, for example, is 0-255)
    tuner.setOutputRange(0, 255);

    // Set the Ziegler-Nichols tuning mode
    // Set it to either PIDAutotuner::ZNModeBasicPID, PIDAutotuner::ZNModeLessOvershoot,
    // or PIDAutotuner::ZNModeNoOvershoot. Defaults to ZNModeNoOvershoot as it is the
    // safest option.
    tuner.setZNMode(PIDAutotuner::ZNModeBasicPID);

    // This must be called immediately before the tuning loop
    // Must be called with the current time in microseconds
    tuner.startTuningLoop(micros());
    tuner.setTuningCycles(50);

    // Run a loop until tuner.isFinished() returns true
    long microseconds;
    while (!tuner.isFinished()) {

        // This loop must run at the same speed as the PID control loop being tuned
        long prevMicroseconds = microseconds;
        microseconds = micros();

        // Get input value here (temperature, encoder position, velocity, etc)
        double input = getRpm();

        // Call tunePID() with the input value and current time in microseconds
        double output = tuner.tunePID(input, microseconds);

        // Set the output - tunePid() will return values within the range configured
        // by setOutputRange(). Don't change the value or the tuning results will be
        // incorrect.
        setRpm(output);

        // This loop must run at the same speed as the PID control loop being tuned
        while (micros() - microseconds < 100000) delayMicroseconds(1);
    }

    // Turn the output off here.
    setRpm(0);

    // Get PID gains - set your PID controller's gains to these
    k_p = tuner.getKp();
    k_i = tuner.getKi();
    k_d = tuner.getKd();
    Serial.print("k_p: "); Serial.println(k_p);
    Serial.print("k_i: "); Serial.println(k_i);
    Serial.print("k_d: "); Serial.println(k_d);
    motorPID.SetTunings(k_p, k_i, k_d);
    delay(1500);

// k_p: 5.81
// k_i: 0.98
// k_d: 9.84

// k_p: 5.53
// k_i: 0.74
// k_d: 11.16

// k_p: 27.00
// k_i: 1056.73
// k_d: 9.21

// k_p: 5.65
// k_i: 0.88
// k_d: 9.68

// k_p: 5.42
// k_i: 0.74
// k_d: 10.63

// k_p: 5.71
// k_i: 0.90
// k_d: 9.88


}

void loop() {
    if (digitalRead(28)) {
        inputRpm = getRpm();
        motorPID.Compute();
        setRpm(outputRpm);
        // Serial.println(outputRpm);
    } else {
        setRpm(0);
        Serial.print("k_p: "); Serial.println(k_p);
        Serial.print("k_i: "); Serial.println(k_i);
        Serial.print("k_d: "); Serial.println(k_d);
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

void setRpm(double outputRpm) //* rev min^-1
{
    if (outputRpm >= 0) {
        digitalWrite(pin1, LOW);
        analogWrite(pin2, (int)(outputRpm)); 
    } else {
        analogWrite(pin1, (int)(outputRpm));
        digitalWrite(pin2, LOW);
    }
}
