#include <Arduino.h>
#include <SoftwareSerial.h>

// https://docs.arduino.cc/learn/built-in-libraries/software-serial

SoftwareSerial mySerial(10,11); // RX, TX

void setup() {
  Serial.begin(9600);
  mySerial.begin(9600);
  pinMode(10, INPUT);
  pinMode(11, OUTPUT);

}

void loop() {

  if (mySerial.available() > 0) {
    // char inChar = (char)mySerial.read();
    String readString = mySerial.readStringUntil('\n');
    Serial.print("Received: ");
    Serial.println(readString);
  }
}
