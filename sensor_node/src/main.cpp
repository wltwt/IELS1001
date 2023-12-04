#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
// #include "Adafruit_HTU21DF.h"
#include <SoftwareSerial.h>

#define LIGHT_SENSOR_PIN_01 A0
#define LIGHT_SENSOR_PIN_02 A1
#define LIGHT_SENSOR_PIN_03 A2
#define LIGHT_SENSOR_PIN_04 A3
#define SCALE_PIN A4
#define NODE_ARRAY_SIZE 4
#define RXpin 3
#define TXpin 2

// Adafruit_HTU21DF htu = Adafruit_HTU21DF();

SoftwareSerial sws(RXpin,TXpin);

int16_t angle;

// references:
// https://www.electronics-lab.com/project/using-sg90-servo-motor-arduino/

Servo servo_01;
Servo servo_02;


float error;
unsigned long timer = 0;
int16_t brightest_node = 0;
int16_t brightest_node_value = 0;
int16_t darkest_node_value = 0;
int16_t darkest_node = 0;
int16_t map_01;
int16_t servo_01_position = 90;
int16_t servo_02_position = 90;

float servo_01_position_float = 90;


enum Servos 
{
  servo1 = 9,
  servo2 = 10  
};

// lys-sensor node
struct Node
{
  uint16_t index;
  int16_t value;
};

Servos e;
Node light_nodes[NODE_ARRAY_SIZE];

void addPosition(Servos s) 
{
  if (s == servo1) 
  {
    if (servo_01_position < 180) 
    {
      servo_01_position++;
    }
  }
  else if (s == servo2) 
  {
    if (servo_02_position < 180)
    {
      servo_02_position++;
    }
  }
}

void subtractPosition(Servos s)  
{
  if (s == servo1) 
  {
    if (servo_01_position > 0) 
    {
      servo_01_position--;
    }
  }
  else if (s == servo2) 
  {
    if (servo_02_position > 0)
    {
      servo_02_position--;
    }
  }
}

void updateNodes() 
{
  light_nodes[0].value = analogRead(LIGHT_SENSOR_PIN_04);
  light_nodes[1].value = analogRead(LIGHT_SENSOR_PIN_03);
  light_nodes[2].value = analogRead(LIGHT_SENSOR_PIN_02);
  light_nodes[3].value = analogRead(LIGHT_SENSOR_PIN_01);
}

void makeNodes()
{
  for (int i = 0; i < NODE_ARRAY_SIZE; i++) {
    light_nodes[i].index = i;
  }
}

void getExtremes()
{
  brightest_node_value = light_nodes[0].value;
  brightest_node = light_nodes[0].index;
  darkest_node = light_nodes[0].index;
  darkest_node_value = light_nodes[0].value;
  
  for (int i = 1; i < NODE_ARRAY_SIZE; i++)
  {  
    if (light_nodes[i].value > brightest_node_value)
    {
      brightest_node = i;
    }
    
    if (light_nodes[i].value < darkest_node_value) 
    {
      darkest_node = i;
    }

    darkest_node_value = light_nodes[darkest_node].value;
    brightest_node_value = light_nodes[brightest_node].value;
  }
}

void positionServos() 
{
  if (brightest_node_value - darkest_node_value > 60) 
  {
    if (3 == brightest_node && 0 == darkest_node) 
    {
      addPosition(servo1);
      subtractPosition(servo2);
    }
    else if (2 == brightest_node && 1 == darkest_node)
    {
      subtractPosition(servo1);
      addPosition(servo2);
    }
    else if (1 == brightest_node && 2 == darkest_node) {
      addPosition(servo1);
      subtractPosition(servo2);
    }
    else if (0 == brightest_node && 3 == darkest_node) {
      subtractPosition(servo1);
      addPosition(servo2);
    }
    servo_02.write(servo_02_position);
    servo_01.write(servo_01_position);
  }
}

void positionServosPID() 
{

  error = map(light_nodes[0].value - light_nodes[1].value, -512, 512, -90, 90);


  // servo_01_position_float = 180 - (map(light_nodes[0].value + light_nodes[1].value, 0, 2046, 0, 180));
  if (light_nodes[0].value - light_nodes[1].value > 60) {
    servo_01_position_float = 90 - 0.8*error;
  }
  int16_t n1 = light_nodes[0].value;
  int16_t n2 = light_nodes[1].value;


  servo_01.write((int)servo_01_position_float);

  Serial.print(">error:");
  Serial.println(error);
  Serial.print(">node 0:");
  Serial.println(n1);
  Serial.print(">node 1:");
  Serial.println(n2);

}

float totalPowerProduced(float temperature) {
  float total = 0;
  for (int i = 0; i < NODE_ARRAY_SIZE; i++) {
    total += light_nodes[i].value; 
  }
  return (total / 4) * (0.3 * temperature);
}


void setup()
{
  Serial.begin(9600);
  pinMode(RXpin, INPUT);
  pinMode(TXpin, OUTPUT);
  servo_01.attach(9);
  servo_02.attach(6);
  makeNodes();
  sws.begin(300);
}


void loop() {
  updateNodes();
  getExtremes();
  if (millis() - timer > 30)
  {
    timer = millis();
    // positionServos();
    // positionServosPID();
  }
  String swReceive = "";

  if (sws.available() > 0) {
    swReceive = sws.readStringUntil('\n');
    Serial.println(swReceive);
  }

  float temp = swReceive.toFloat();
  int16_t weight = analogRead(SCALE_PIN);
  String wp = (String)totalPowerProduced(temp);

  Serial.println("Watts produced: " + wp);

  delay(100);


  // Serial.print(">brightestNode:");
  // Serial.println(brightest_node);
  // Serial.print(">brightestnodevalue:");
  // Serial.println(brightest_node_value);
  // Serial.print(">darkestnode:");
  // Serial.println(darkest_node);
  // Serial.print(">darkest node value:");
  // Serial.println(darkest_node_value);
  // Serial.print(">servo_position:");
  // Serial.println(servo_01_position);
  

  // Serial.print(">read_05_servo:");
  // Serial.println(servo_01.read());

}
