#include <ColorSensorTCS34725.h>

// Manual for library: http://lygte-info.dk/project/ColorSensorTCS34725Library%20UK.html
// By HKJ from lygte-info.dk

#define COLOR_SDA_PIN  12
#define COLOR_SCL_PIN  11


ColorSensorTCS34725 colorSensor(COLOR_SDA_PIN, COLOR_SCL_PIN);

void setup() {
  Serial.begin(9600);
  colorSensor.setWaitTime(1000);  // Slow down conversion
  colorSensor.setIntegrationTime(1000); // Library will limit to maximum possible time
  colorSensor.setGain(CS_GAIN_16);
  colorSensor.begin();
}

long loopCount = 0;

void loop() {
  loopCount++;
// It is possible to use isReady()/clearReady() or isInterrupt()/clearInterrupt()
  if (colorSensor.isReady()) {
    colorSensor.clearReady();
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