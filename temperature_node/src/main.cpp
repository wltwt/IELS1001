#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include "Adafruit_HTU21DF.h"
#include <ArduinoJson.h>

// https://docs.arduino.cc/learn/built-in-libraries/software-serial

SoftwareSerial sws(10, 11); // RX, TX
Adafruit_HTU21DF htu = Adafruit_HTU21DF();

StaticJsonDocument<40> doc;


void setup()  
{
  // Serial.begin(9600);
  sws.begin(300);
  pinMode(10, INPUT);
  pinMode(11, OUTPUT);
  htu.begin();
}

void loop()
{
  float temp = htu.readTemperature();
  float humidity = htu.readHumidity();

  doc["temp"] = temp;
  doc["humidity"] = humidity;

  serializeJson(doc, sws);

  delay(500);
}