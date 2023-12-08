#include <Arduino.h>
#include <Servo.h>
#include <Wire.h>
// #include <SoftwareSerial.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <SPI.h>


#define LIGHT_SENSOR_PIN_01 A0
#define LIGHT_SENSOR_PIN_02 A1
#define LIGHT_SENSOR_PIN_03 A2
#define LIGHT_SENSOR_PIN_04 A3
#define SCALE_PIN A4
#define NODE_ARRAY_SIZE 4
#define HUMIDITY_ARRAY_SIZE 50
// #define RXpin 4
// #define TXpin 2

// Adafruit_HTU21DF htu = Adafruit_HTU21DF();

// SoftwareSerial sws(RXpin,TXpin);
StaticJsonDocument<40> doc;

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
float effect_output;
float temp;
float humidity;
float servo_01_position_float = 90;

// get from eeprom
float solar_panel_health = 100;




enum Servos 
{
  servo1 = 5,
  servo2 = 6  
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

void updateLightNodes() 
{
  light_nodes[0].value = analogRead(LIGHT_SENSOR_PIN_04);
  light_nodes[1].value = analogRead(LIGHT_SENSOR_PIN_03);
  light_nodes[2].value = analogRead(LIGHT_SENSOR_PIN_02);
  light_nodes[3].value = analogRead(LIGHT_SENSOR_PIN_01);
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

void getEffect() 
{
  // String sw_receive = "";
  // if (sws.available() > 0) 
  // {
    // sw_receive = sws.readStringUntil('\n');
    // temp = sw_receive.toFloat();
  // }

  // effect_output = totalPowerProduced(temp);
  // Serial.println(effect_output);
}


void updateSolarCellLifeSpan(float humidity) 
{
  static float average_readings[HUMIDITY_ARRAY_SIZE];
  static byte count = 0;
  static unsigned long solar_call_timer = 0;

  if ( (millis() - solar_call_timer > 10))
  {
    solar_call_timer = millis(); 
    average_readings[count] = humidity;

    count++;

    Serial.println(count);
    if (HUMIDITY_ARRAY_SIZE == count)
    {
      float average_total_readings;

      for (int i = 0; i < HUMIDITY_ARRAY_SIZE; i++) 
      {
        average_total_readings += average_readings[i];
      }

      average_total_readings = average_total_readings / HUMIDITY_ARRAY_SIZE;

      if (average_total_readings > 0)
      {
        solar_panel_health -= average_total_readings * 0.001;
        // write ee-prom?
        Serial.println(solar_panel_health);
      }


      // EEPROM.put(0x00, solar_panel_health);
      count = 0;
    }
  }
} 

volatile byte dataReceived;
// volatile bool data_received = false;

void setup()
{
  Serial.begin(9600);

  // Serial.begin(9600);
  // pinMode(RXpin, INPUT);
  // pinMode(TXpin, OUTPUT);
  // servo_01.attach(servo1);
  // servo_02.attach(servo2);
  // makeNodes();

  pinMode(MISO, OUTPUT);

  SPI.begin();

  SPI.attachInterrupt();
  // sws.begin(8000);
}

byte respondSPI(byte data_received)
{
  return data_received + 1;
}

ISR (SPI_STC_vect) {
  dataReceived = SPDR;
  SPDR = respondSPI(dataReceived);
}


void loop() 
{
  // Serial.println(data_received);
  // updateLightNodes();
  // getExtremes();
  // getEffect();
  // delay(1000);

  // if (data_received) 
  // {
    // Serial.begin(9600);
    // Serial.println(received);
    // Serial.end();
    // data_received = false;
  // }

  if (millis() - timer > 100)
  {
    timer = millis();
    // Serial.println(received);
      // timer = millis();
      // DeserializationError error = deserializeJson(doc, sws);
      // if (error) {
      // Serial.print(F("deserializeJson() failed: "));
      // Serial.println(error.f_str());
      // return;
      // }
    // positionServos();
    // positionServosPID();
  }

  int16_t weight = analogRead(SCALE_PIN);

  // // String wp = (String)effect_output;
  // if (sws.available() > 0) {
  //   String json = sws.readStringUntil('\n'); 
  //   Serial.println(json);
  // }

  

  // if (error) {
  //   Serial.print(F("deserializeJson() failed: "));
  //   Serial.println(error.f_str());
  //   return;
  // }

  // temp = doc["temp"];
  // humidity = doc["humidity"];
  // effect_output = totalPowerProduced(temp);
  // Serial.println("Humidity: " + (String)humidity);
  // Serial.println("Temmperature: " + (String)temp);
  // Serial.println(effect_output);


  // updateSolarCellLifeSpan(humidity);
  // delay(100);


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
