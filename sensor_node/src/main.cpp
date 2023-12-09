#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Adafruit_HTU21DF.h"

#define LIGHT_SENSOR_PIN A0
#define SCALE_PIN A1
#define NODE_ARRAY_SIZE 4
#define HUMIDITY_ARRAY_SIZE 50

enum Servos 
{
  servo1 = 10,
  servo2 = 9  
};

// lys-sensor node
struct Node
{
  uint16_t index;
  int16_t value;
};


Adafruit_HTU21DF htu = Adafruit_HTU21DF();
Servos e;
Node light_nodes[NODE_ARRAY_SIZE];
Servo servo_01;
Servo servo_02;


int16_t angle;

// references:
// https://www.electronics-lab.com/project/using-sg90-servo-motor-arduino/



float error;
unsigned long timer = 0;
int16_t brightest_node = 0;
int16_t brightest_node_value = 0;
int16_t darkest_node_value = 0;
int16_t darkest_node = 0;
int16_t map_01;
int16_t servo_01_position = 90;
int16_t servo_02_position = 90;
float effect_output;
float temp;
float humidity;
float servo_01_position_float = 90;

float prev_powerProduced = 0;
float power_produced;

float average_readings[HUMIDITY_ARRAY_SIZE];


// get from eeprom
float solar_panel_health = 0.0f;




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

void resetAll() 
{
  delayMicroseconds(5);
  for (int i = 0; i < NODE_ARRAY_SIZE; i++)
  {
    digitalWrite(i+2, LOW);
  }
  delayMicroseconds(5);
}

void updateLightNodes()
{
  for (int i = 0; i < NODE_ARRAY_SIZE; i++)
  {
    resetAll();
    digitalWrite(i+2, HIGH);
    light_nodes[i].value = map(-70,1023,0,1023,analogRead(LIGHT_SENSOR_PIN));
  }
}

void makeNodes()
{
  for (int i = 0; i < NODE_ARRAY_SIZE; i++) 
  {
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
  // Serial.println(">brightestNode:" + (String)brightest_node);
}

void positionServos() 
{
  if (brightest_node_value - darkest_node_value > 50) 
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
    else if (1 == brightest_node && 2 == darkest_node) 
    {
      addPosition(servo1);
      subtractPosition(servo2);
    }
    else if (0 == brightest_node && 3 == darkest_node) 
    {
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
  if (light_nodes[0].value - light_nodes[1].value > 60)
  {
    servo_01_position_float = 90 - 0.8 * error;
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

float totalPowerProduced(float temperature) 
{
  float total = 0;

  for (int i = 0; i < NODE_ARRAY_SIZE; i++) 
  {
    total += light_nodes[i].value; 
  }

  // gi mindre effekt med -0.3C for hver grad over 25
  if (temperature > 25.0) 
  {
    return (total / 4) * (1 - ( (temperature - 25) * 0.03));
  }
  else
  {
    return total / 4;
  }
}

void updateSolarCellLifeSpan(float humidity) 
{
  static byte count = 0;
  static unsigned long solar_call_timer = 1000;

  // sikring for EEPROM
  if ( (millis() - solar_call_timer > 100))
  {
    solar_call_timer = millis(); 
    average_readings[count] = humidity;

    count++;

    Serial.println(">count: " + (String)count);
    if (HUMIDITY_ARRAY_SIZE == count)
    {
      float average_total_readings;

      for (int i = 0; i < HUMIDITY_ARRAY_SIZE; i++)
      {
        average_total_readings += average_readings[i];
      }

      average_total_readings /= HUMIDITY_ARRAY_SIZE;

      if (average_total_readings > 0 && average_total_readings < 50)
      {
        solar_panel_health -= (average_total_readings * 0.001);
        EEPROM.put(0x00, solar_panel_health);
      }
      count = 0;
    }
  }
}


void setup()
{
  Serial.begin(9600);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);

  servo_01.attach(servo1);
  servo_02.attach(servo2);
  makeNodes();
  // solar_panel_health = 100;

  // EEPROM.put(0x0, solar_panel_health);
  
  
  EEPROM.get(0x0, solar_panel_health);
  // solar_panel_health = 100;

  htu.begin();

} 


void loop() 
{
  updateLightNodes();
  getExtremes();

  if (millis() - timer > 1000)
  {
    timer = millis();
    // unsigned long timeattack = micros();
    // ~100ms på å lese
    float temp = htu.readTemperature();
    float rel_hum = htu.readHumidity();

    updateSolarCellLifeSpan(temp);
    // timeattack = micros() - timeattack;
    // Serial.println(">Time:" + (String)timeattack);
    
    prev_powerProduced = power_produced;

    power_produced = totalPowerProduced(temp);




    // Serial.println(received);
    // positionServos();
    // positionServosPID();
    int16_t weight = analogRead(SCALE_PIN);

    // Serial.println(">1 Node value: " + (String)light_nodes[0].value);
    // Serial.println(">2 Node value: " + (String)light_nodes[1].value);
    // Serial.println(">3 Node value: " + (String)light_nodes[2].value);
    // Serial.println(">4 Node value: " + (String)light_nodes[3].value);

    // Serial.println(">Temmperature: " + (String)temp);
    // Serial.println(">Weight: " + (String)weight);

  }

  Serial.println(">powerProducedprevious: " + (String)(prev_powerProduced - power_produced));
  Serial.println(">Health: " + (String)solar_panel_health);




  // if (prev_powerProduced - power_produced >= 10); 
  // {
    positionServos();
  // }




  // effect_output = totalPowerProduced(temp);
  // updateSolarCellLifeSpan(humidity);
  // delay(100);
}
