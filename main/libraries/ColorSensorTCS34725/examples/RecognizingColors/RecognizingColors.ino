#include <ColorSensorTCS34725.h>

// Manual for library: http://lygte-info.dk/project/ColorSensorTCS34725Library%20UK.html
// By HKJ from lygte-info.dk

/*
  To use in reflective mode (Works best with low ambient light):
  White led must be on (LED module pin unconnected or connected to a HIGH Arduino pin)
  Place white paper over sensor
  Reset Arduino
  Serial terminal will start listing values around 10000 (8000-12000)
  Remove white paper
  Place any color item above the sensor and it will measure the color

To measure light color
  White led must be off (LED module pin connected to GND or a LOW Arduino pin)
  Illuminate sensor with white light at about same brightness as lists to test.
  Reset Arduino
  Serial terminal will start listing values around 10000 (8000-12000)
  Remove white light
  Aim colored light at sensor and it will measure the color
*/

#define COLOR_SDA_PIN  12
#define COLOR_SCL_PIN  11

ColorSensorTCS34725 colorSensor(COLOR_SDA_PIN, COLOR_SCL_PIN);

RGBC colorRef;

void setup() {
  Serial.begin(9600);
  colorSensor.setWaitTime(0);
  colorSensor.setIntegrationTime(200);
  colorSensor.setGain(CS_GAIN_16);
  colorSensor.begin();

  delay(2000);
  colorRef = colorSensor.readRGBC();
}

void loop() {
  if (colorSensor.isReady()) {
    colorSensor.clearReady();
    RGBC v = colorSensor.readRGBCRelative(colorRef);
    Serial.print("Red: ");
    Serial.print(v.r);
    Serial.print("   Green: ");
    Serial.print(v.g);
    Serial.print("   Blue: ");
    Serial.print(v.b);
    Serial.print("   Clear: ");
    Serial.print(v.c);
    Serial.print("   Hue: ");
    int hue = colorSensor.calcHue(v);
    Serial.print(hue);
    if (hue >= 330 || (hue>=0 && hue < 30)) Serial.print(" red");
    if (hue >= 30 && hue < 90) Serial.print(" yellow");
    if (hue >= 90 && hue < 150) Serial.print(" green");
    if (hue >= 150 && hue < 210) Serial.print(" cyan");
    if (hue >= 210 && hue < 270) Serial.print(" blue");
    if (hue >= 270 && hue < 330) Serial.print(" magenta");
    Serial.println();
    delay(2000);
  }
}