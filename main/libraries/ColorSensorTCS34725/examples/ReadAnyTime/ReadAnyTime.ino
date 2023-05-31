#include <ColorSensorTCS34725.h>

// Manual for library: http://lygte-info.dk/project/ColorSensorTCS34725Library%20UK.html
// By HKJ from lygte-info.dk

#define COLOR_SDA_PIN  12
#define COLOR_SCL_PIN  11

ColorSensorTCS34725 colorSensor(COLOR_SDA_PIN, COLOR_SCL_PIN);

void setup() {
  Serial.begin(9600);
  colorSensor.setWaitTime(0);
  colorSensor.setIntegrationTime(200);
  colorSensor.setGain(CS_GAIN_16);
  colorSensor.begin();
}

void loop() {
  RGBC v = colorSensor.readRGBC();  // The sensor can be read at any time
  Serial.print("Red: ");
  Serial.print(v.r);
  Serial.print("   Green: ");
  Serial.print(v.g);
  Serial.print("   Blue: ");
  Serial.print(v.b);
  Serial.print("   Clear: ");
  Serial.print(v.c);
  Serial.println();
  delay(2000);

}
