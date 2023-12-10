#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
#include <EEPROM.h>
#include "Adafruit_HTU21DF.h"
#include <ArduinoJson.h>

#define LIGHT_SENSOR_PIN A0
#define SCALE_PIN A1
#define NODE_ARRAY_SIZE 4
#define HUMIDITY_ARRAY_SIZE 50
#define TEMP_ARRAY_SIZE 50
#define ONE_SECOND_MS 1000


enum Servos 
{
  first_servo = 10,
  second_servo = 9  
};

enum LDRpin
{
  first_pin = 2,
  last_pin = 5
};

// lys-sensor node
struct LDRnode
{
  uint16_t index;
  int16_t value;
};

Adafruit_HTU21DF htu = Adafruit_HTU21DF();
Servos e;
LDRnode light_nodes[NODE_ARRAY_SIZE];
Servo servo_one;
Servo servo_two;

// references:
// https://www.electronics-lab.com/project/using-sg90-servo-motor-arduino/
// https://docs.arduino.cc/learn/programming/eeprom-guide


float error;
unsigned long timer = 0;
unsigned long update_LDR_timer = 0;
int16_t brightest_node = 0;
int16_t brightest_node_value = 0;
int16_t darkest_node_value = 0;
int16_t darkest_node = 0;
int16_t first_servo_position = 90;         // startposisjon servo 1
int16_t second_servo_position = 90;         // startposisjon servo 2
float effect_output;
float temp;
float humidity;
float servo_one_position_float = 90;

float prev_powerProduced = 0;
float power_produced;

float humidity_readings[HUMIDITY_ARRAY_SIZE];
float temp_readings[TEMP_ARRAY_SIZE];
float solar_panel_health = 0.0f;                // henter fra EEPROM

/*
*  oppdaterer globale server-posisjon verdier
*  sørger for at 180 grader ikke blir overstridd
*/

void addPosition(Servos s)
{
  if (s == first_servo) 
  {
    if (first_servo_position < 180) 
    {
      first_servo_position++;
      servo_one.write(first_servo_position);
    }
  }
  else if (s == second_servo) 
  {
    if (second_servo_position < 180)
    {
      second_servo_position++;
      servo_two.write(second_servo_position);
    }
  }
}

/*
*  oppdaterer globale server-posisjon verdier
*  sørger for at 180 grader ikke blir overstridd
*/

void subtractPosition(Servos s)  
{
  if (s == first_servo) 
  {
    if (first_servo_position > 0) 
    {
      first_servo_position--;
      servo_one.write(first_servo_position);
    }
  }
  else if (s == second_servo) 
  {
    if (second_servo_position > 0)
    {
      second_servo_position--;
      servo_two.write(second_servo_position);
    }
  }
  
}

/*
*  oppdaterer globale server-posisjon verdier
*  sørger for at 180 grader ikke blir overstridd
*/

void resetAll() 
{
  delayMicroseconds(10);
  for (int pin = first_pin; pin <= last_pin; pin++)
  {
    digitalWrite(pin, LOW);
  }
  delayMicroseconds(10);
}

/*
*  oppdaterer globale server-posisjon verdier
*  sørger for at 180 grader ikke blir overstridd
*/

void updateLightNodes()
{
  for (int i = 0; i < NODE_ARRAY_SIZE; i++)
  {
    resetAll();
    digitalWrite(i+2, HIGH);
    light_nodes[i].value = map(-70,1023,0,1023,analogRead(LIGHT_SENSOR_PIN));
  }
}

/*
*  setter index til lys-sensor nodene
*/

void makeNodes()
{
  for (int i = 0; i < NODE_ARRAY_SIZE; i++) 
  {
    light_nodes[i].index = i;
  }
}

/*
*  finn hvilken lys-sensor som gir høyest
*  og minst utslag
*/

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

/*
*  posisjoner servoer basert på hvilken
*  lys-sensor som er lysest og mørkest
*/

void positionServos() 
{
  if (brightest_node_value - darkest_node_value > 50) 
  {
    if (3 == brightest_node && 0 == darkest_node) 
    {
      addPosition(first_servo);
      subtractPosition(second_servo);
    }
    else if (2 == brightest_node && 1 == darkest_node)
    {
      subtractPosition(first_servo);
      addPosition(second_servo);
    }
    else if (1 == brightest_node && 2 == darkest_node) 
    {
      addPosition(first_servo);
      subtractPosition(second_servo);
    }
    else if (0 == brightest_node && 3 == darkest_node) 
    {
      subtractPosition(first_servo);
      addPosition(second_servo);
    }
  }
}

// fungerer ikke enda
void positionServosPID() 
{
  error = map(light_nodes[0].value - light_nodes[1].value, -512, 512, -90, 90);

  // servo_one_position_float = 180 - (map(light_nodes[0].value + light_nodes[1].value, 0, 2046, 0, 180));
  if (light_nodes[0].value - light_nodes[1].value > 60)
  {
    servo_one_position_float = 90 - 0.8 * error;
  }
  int16_t n1 = light_nodes[0].value;
  int16_t n2 = light_nodes[1].value;


  servo_one.write((int)servo_one_position_float);

  Serial.print(">error:");
  Serial.println(error);
  Serial.print(">node 0:");
  Serial.println(n1);
  Serial.print(">node 1:");
  Serial.println(n2);
}

/*
*  Returnerer en float med gjennomsnittet
*  av de fire sensorene sine verdier.
*/

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

/*
*  Oppdaterer solcellepanelets helsetilstand basert 
*  på gjennomsnittet av de 50 siste avlesningene.
*/

void updateSolarCellLifeSpan(float humidity, float temperature) 
{
  static byte count = 0;
  static unsigned long solar_call_timer = ONE_SECOND_MS;

  // sikring for EEPROM-skriving
  if ( (millis() - solar_call_timer) > 100)
  {
    solar_call_timer = millis(); 
    humidity_readings[count] = humidity;

    count++;

    if (HUMIDITY_ARRAY_SIZE == count)
    {
      float average_total_readings;

      for (int i = 0; i < HUMIDITY_ARRAY_SIZE; i++)
      {
        average_total_readings += humidity_readings[i];
      }

      average_total_readings = average_total_readings / HUMIDITY_ARRAY_SIZE;

      if (average_total_readings > 0 && average_total_readings < 50)
      {
        solar_panel_health -= (average_total_readings * 0.001);
        EEPROM.put(0x00, solar_panel_health);
      }

      count = 0;
    
    }
  }
}

/*
*  sjekker om det er trykk på solcellepanelet og gjør en
*  bevegelse for å få av det som skaper trykket (snø f.eks)
*/

void checkPressure(int16_t pressure) 
{
  int16_t first_servo_temp_position = first_servo_position;
  int16_t second_servo_temp_position = second_servo_position;
  
  if (pressure > 500) 
  {
    // gå til originalposisjon
    if (first_servo_position > 90) 
    {
      while (first_servo_position > 90)
      {
        subtractPosition(first_servo);
        delayMicroseconds(5000);
      }
    }
    else
    {
      while (first_servo_position < 90)
      {
        addPosition(first_servo);
        delayMicroseconds(5000);
      }
    }

    if (second_servo_position > 90)
    {
      while (second_servo_position > 90)
      {
        subtractPosition(second_servo);
        delayMicroseconds(5000);
      }
    }
    else 
    {
      while (second_servo_position < 90)
      {
        addPosition(second_servo);
        delayMicroseconds(5000);

      }
    }

    delay(100);

    int position;
    for (position = first_servo_position; position < 180; position++)
    {
      servo_one.write(position);
      delayMicroseconds(5000);
    }

    for (position = position; position > 90; position--)
    {
      servo_one.write(position);
      delayMicroseconds(10000);
    }
    delay(100);
    servo_one.write(first_servo_temp_position);
    servo_two.write(second_servo_temp_position);
  }
}

/*
*  send info til dashboard
*/

void sendData(float temp, float humidity)
{
  static unsigned long tmr = 10000;
  if (millis() - tmr > 10000)
  {
    tmr = millis();
    StaticJsonDocument<200> doc;
    doc["temp"] = temp;
    doc["humidity"] = humidity;
    doc["health_level"] = solar_panel_health;
    doc["power_produced"] = power_produced;

    String json;
    serializeJson(doc, json);

    Serial.println(json);
  }
}

void setup()
{
  Serial.begin(9600);
  for (int pin = first_pin; pin <= last_pin; pin++)
  {
    pinMode(pin, OUTPUT);
  }

  servo_one.attach(first_servo);
  servo_two.attach(second_servo);

  makeNodes();
  // solar_panel_health = 100;
  // EEPROM.put(0x0, solar_panel_health);

  EEPROM.get(0x0, solar_panel_health);
  htu.begin();
}

void loop() 
{
  if (millis() - update_LDR_timer > 50)
  {
    update_LDR_timer = millis();
    updateLightNodes();
    getExtremes();
    positionServos();
  }

  if (millis() - timer > ONE_SECOND_MS)
  {
    timer = millis();
    // ~100ms på å lese
    temp = htu.readTemperature();
    humidity = htu.readHumidity();
    
    updateSolarCellLifeSpan(humidity, temp);
    prev_powerProduced = power_produced;
    power_produced = totalPowerProduced(temp);
    
    int16_t solar_panel_pressure = analogRead(SCALE_PIN);
    checkPressure(solar_panel_pressure);
  }

  sendData(temp, humidity);

}
