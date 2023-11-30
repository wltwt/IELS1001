#include <Arduino.h>

#define LIGHT_SENSOR_PIN_01 A0
#define LIGHT_SENSOR_PIN_02 A1
#define LIGHT_SENSOR_PIN_03 A2
#define LIGHT_SENSOR_PIN_04 A3


void setup() {
  Serial.begin(9600);
}

void loop() {

  int16_t read_01 = analogRead(LIGHT_SENSOR_PIN_01);
  int16_t read_02 = analogRead(LIGHT_SENSOR_PIN_02);
  int16_t read_03 = analogRead(LIGHT_SENSOR_PIN_03);
  int16_t read_04 = analogRead(LIGHT_SENSOR_PIN_04);

  Serial.print(">read_01:");
  Serial.println(read_01);

  Serial.print(">read_02:");
  Serial.println(read_02);

  Serial.print(">read_03:");
  Serial.println(read_03);

  Serial.print(">read_04:");
  Serial.println(read_04);
  
}
