#include <Arduino.h>
// #include <SoftwareSerial.h>
// #include <Wire.h>
// #include "Adafruit_HTU21DF.h"
// #include <ArduinoJson.h>
#include <SPI.h>

// https://docs.arduino.cc/learn/built-in-libraries/software-serial
// https://arduinojson.org/v6/example/generator/
// https://microcontrollerslab.com/spi-communication-between-two-arduino-boards/
// https://docs.arduino.cc/tutorials/generic/introduction-to-the-serial-peripheral-interface
// https://docs.arduino.cc/hacking/software/PortManipulation


// SoftwareSerial sws(10, 11); // RX, TX
// Adafruit_HTU21DF htu = Adafruit_HTU21DF();

// StaticJsonDocument<40> doc;


// void setup()  
// {
//   SPI.begin();
//   Serial.begin(9600);
//   SPI.setClockDivider(SPI_CLOCK_DIV8);
//   pinMode(SS, OUTPUT);
  


  // Serial.begin(9600);
  // sws.begin(1152200);
  // pinMode(10, INPUT);
  // pinMode(11, OUTPUT);
  // htu.begin();
// }

// void loop()
// {

  // byte b_send = 0b01010101;
  // byte b_receive;
  // Serial.print("Test");


  // float temp = htu.readTemperature();
  // float humidity = htu.readHumidity();
// 
  // doc["temp"] = temp;
  // doc["humidity"] = humidity;


 




//   delay(1000);
// }


volatile byte dataEcho;  //echo back input data in next round
volatile byte dataToPC;  //send input data to PC

void setup() {
    // Serial.begin(115200);  //link to PC
    
    //The Port B Data Direction Register
    DDRB  |= 0b00010000; 
   //The Port B 
    PORTB |= 0b00000100;
    
    //SPI Control Register
    SPCR  |= 0b11000000;
    //SPI status register
    SPSR  |= 0b00000000;
    
    dataEcho = 0;
    dataToPC = 0;
    
    sei();
}

// void setup() {
//   Serial.begin(115200);
//   pinMode(MISO, OUTPUT); 
//   pinMode(SS, INPUT); 
//   // digitalWrite(SS, HIGH);




//   SPI.begin();

//   SPI.attachInterrupt();

//   // dataEcho = 0;
//   // dataToPC = 0;

//   // sei();
// }

void loop() {
  
  // if(dataToPC != 0){
  //   Serial.write(dataToPC);
  //   dataToPC = 0;
  // }

}

ISR(SPI_STC_vect){
  cli();
  
  //while SS Low
  while(!(PINB & 0b00000100)){
    SPDR = dataEcho;
    
    //wait SPI transfer complete
    while(!(SPSR & (1 << SPIF)));
    
    dataEcho = SPDR;  //send back in next round
  }
  sei();
}


// ISR(SPI_STC_vect) {
//   dataEcho = SPDR;  // Read the received data
//   SPDR = dataEcho;  // Echo back the received data
// }

// byte processReceivedData(byte data) {
//   // Example: simply echo the received data
//   return data;
// }

// volatile byte receivedData;
// volatile bool dataReady = false;

// void setup() {
//   Serial.begin(115200);  // Start serial communication for debugging
//   pinMode(MISO, OUTPUT); // Set MISO (Master In Slave Out) as output

//   SPI.attachInterrupt(); // Enable SPI interrupt
//   SPI.begin();           // Initialize SPI in slave mode
// }

// ISR(SPI_STC_vect) {
//   cli();
//   byte receivedData = SPDR;  // Read the received data
//   // Process the received data to determine the response
//   byte responseData = processReceivedData(receivedData);
  
//   // Wait for any ongoing SPI transmission to complete
//   while(!(SPSR & (1 << SPIF)));

//   SPDR = responseData; // Set the next byte to be sent
//   sei();
// }


// void loop() {

// }

