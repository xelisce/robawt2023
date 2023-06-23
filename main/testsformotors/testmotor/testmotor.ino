#include <Arduino.h>
#include <Wire.h>
#include "Vroom.h"
#include <Servo.h>
#include "hardware/pio.h"
#include "quadrature.pio.h"

#define QUADRATURE_A_PIN 0
#define QUADRATURE_B_PIN 1

#define default_serial 1
#define test_pi 0
#define test_teensy 0
#define ONBOARDLEDPIN 25

const int TX1PIN = 8,
  RX1PIN = 9,
  TX0PIN = 16,
  RX0PIN = 17;

bool flipDir;
double steer;

// PIO pio = pio0;
// uint offset, sm;

Motor MotorL(12, 13, 1, 0); 
Motor MotorR(11, 10, 19, 18);
  
void ISRLA() { MotorL.readEncA(); }
void ISRLB() { MotorL.readEncB(); }
void ISRRA() { MotorR.readEncA(); }
void ISRRB() { MotorR.readEncB(); }

void setup() 
{
    //^ init the encoder thing
    offset = pio_add_program(pio, &quadrature_program);
    sm = pio_claim_unused_sm(pio, true);
    quadrature_program_init(pio, sm, offset, QUADRATURE_A_PIN, QUADRATURE_B_PIN);

    flipDir = true;
    #if default_serial
    Serial.begin(115200);
    Serial.println("USB Serial initialised!");
    #endif
    pinMode(28, INPUT);
    attachInterrupt(MotorL.getEncAPin(), ISRLA, RISING);
    attachInterrupt(MotorL.getEncBPin(), ISRLB, RISING);
    attachInterrupt(MotorR.getEncAPin(), ISRRA, RISING);
    attachInterrupt(MotorR.getEncBPin(), ISRRB, RISING);
}

long start, stop;
long long previousMicros = 0, currentMicros = 0;
double rpm;
void loop() 
{
    currentMicros = micros();
    if (digitalRead(28)){
        start = millis();
        steer = flipDir ? 30 : -30;
        // Serial.println("Still running!");
        // Serial.println(MotorL.setRpm(steer));
        
        //^ get encoder vals 
        pio_sm_exec_wait_blocking(pio, sm, pio_encode_in(pio_x, 32));
        uint prev_encoder_val = pio_sm_get_blocking(pio, sm); Serial.println(prev_encoder_val);

        if (previousMicros - currentMicros > 500) { //^ if past a certain interval
            previousMicros = currentMicros;
            uint current_encoder_val = pio_sm_get_blocking(pio, sm); Serial.println(prev_encoder_val);
            rpm = (current_encoder_val - prev_encoder_val) / (374) * 2 * 60; //^ times 2 cuz 500ms
            Serial.print("rpm: "); Serial.println(rpm);
            prev_encoder_val = current_encoder_val;
        }
        Serial.println(MotorL.setRpmDirectly(steer, rpm));
        stop = millis();
        Serial.print("Loop time: "); Serial.println(stop - start);
    }
    else {
        flipDir = !flipDir; //^ flips dir on turning off switch
        Serial.println("Stopped");
        Serial.println(MotorL.setRpm(0));
        // Serial.println("Running backwards!");
        // Serial.println(MotorL.setRpm(-30));
    }
}