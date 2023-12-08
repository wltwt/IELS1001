#include <Arduino.h>
#include <SPI.h>
byte dataOut;
byte dataIn;


// https://arduino-er.blogspot.com/2014/09/communication-betweeen-arduinos-using.html
int pinSS = 10;  //Slave Select, active LOW

void setup(){
  Serial.begin(115200);  //link to PC
  
  pinMode(pinSS, OUTPUT);
  digitalWrite(pinSS, HIGH);
  SPI.begin();
}

byte spiWriteAndRead(byte dout){
  byte din;
  digitalWrite(pinSS, LOW);
  delay(1);
  din = SPI.transfer(dout);
  digitalWrite(pinSS, HIGH);
  return din;
}

// void loop(){
//   while(Serial.available() > 0){
//     dataOut = Serial.read();
//     dataIn = spiWriteAndRead(dataOut);
//     Serial.write(dataIn);
//   }
// }

void loop() {
  byte dataToSend = 0x42;  // Example data to send
  byte receivedData;

  digitalWrite(pinSS, LOW);  // Activate the slave device
  receivedData = SPI.transfer(dataToSend);  // Send data and receive response
  digitalWrite(pinSS, HIGH); // Deactivate the slave device

  // Debugging output
  Serial.print("Sent: ");
  Serial.print(dataToSend, HEX);
  Serial.print(", Received: ");
  Serial.println(receivedData, HEX);

  delay(1000);  // Wait for a second
}

