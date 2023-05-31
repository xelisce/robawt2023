#include <ColorSensorTCS34725.h>

// Manual for library: http://lygte-info.dk/project/ColorSensorTCS34725Library%20UK.html
// By HKJ from lygte-info.dk

#define COLOR_SDA_PIN  12
#define COLOR_SCL_PIN  11
#define COLOR_INTERRUPT_PIN 2

ColorSensorTCS34725 colorSensor(COLOR_SDA_PIN, COLOR_SCL_PIN);
boolean colorReady = false;


void colorInterrupt() {
  colorReady = true;
}

void setup() {
  Serial.begin(9600);
  colorSensor.setWaitTime(1000);  // Slow down conversion
  colorSensor.setIntegrationTime(1000); // Library will limit to maximum possible time
  colorSensor.setGain(CS_GAIN_16);
  colorSensor.begin();
  pinMode(COLOR_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(COLOR_INTERRUPT_PIN), colorInterrupt, FALLING);
  colorSensor.interruptPinEnable(true);
}

long loopCount = 0;

void loop() {
  loopCount++;
  if (colorReady) {
    colorSensor.clearInterrupt();
    colorReady = false;
    RGBC v = colorSensor.readRGBC();
    Serial.print("Red: ");
    Serial.print(v.r);
    Serial.print("   Green: ");
    Serial.print(v.g);
    Serial.print("   Blue: ");
    Serial.print(v.b);
    Serial.print("   Clear: ");
    Serial.print(v.c);
    Serial.print("     loop was run: ");
    Serial.print(loopCount);
    Serial.println(" times");
    loopCount = 0;
  }
}