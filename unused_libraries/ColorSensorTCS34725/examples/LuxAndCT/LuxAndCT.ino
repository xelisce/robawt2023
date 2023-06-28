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

void loop() {
  if (colorSensor.isReady()) {
    colorSensor.clearReady();
    RGBC v = colorSensor.readRGBC();
    LightValues lv = colorSensor.calcLight(v);
    Serial.print("Lux: ");
    Serial.println(lv.lux, 0);
    Serial.print("CT: ");
    Serial.println(lv.ct);
    Serial.println();
    delay(2000);
  }
}