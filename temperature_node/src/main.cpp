#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Adafruit_HTU21DF.h"

// https://docs.arduino.cc/learn/built-in-libraries/software-serial

SoftwareSerial mySerial(10, 11); // RX, TX
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

void setup()  
{
  // Serial.begin(9600);
  mySerial.begin(300);
  pinMode(10, INPUT);
  pinMode(11, OUTPUT);

  Serial.println("HTU21D-F test");
  htu.begin();
}

void loop()
{
  float temp = htu.readTemperature();
  // mySerial.write('A');
  // Serial.println(temp);
  // digitalWrite(11, LOW);
  // delay(400);
  // Serial.println("Test");
  mySerial.println(temp);
  // digitalWrite(11, HIGH);
  delay(400);
}