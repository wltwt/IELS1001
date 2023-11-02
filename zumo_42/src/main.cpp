/* This demo shows how the Zumo can use its gyroscope to detect
when it is being rotated, and use the motors to resist that
rotation.

This code was tested on a Zumo with 4 NiMH batteries and two 75:1
HP micro metal gearmotors.  If you have different batteries or
motors, you might need to adjust the PID constants.

Be careful to not move the robot for a few seconds after starting
it while the gyro is being calibrated.  During the gyro
calibration, the yellow LED is on and the words "Gyro cal" are
shown on the display.

After the gyro calibration is done, press button A to start the
demo.  If you try to turn the Zumo, or put it on a surface that
is turning, it will drive its motors to counteract the turning.

This demo only uses the Z axis of the gyro, so it is possible to
pick up the Zumo, rotate it about its X and Y axes, and then put
it down facing in a new position. */

#include <Wire.h>
#include <Zumo32U4.h>


// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const int16_t maxSpeed = 400;

// Change next line to this if you are using the older Zumo 32U4
// with a black and green LCD display:
// Zumo32U4LCD display;
Zumo32U4OLED display;

Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;

Zumo32U4Motors motors;
Zumo32U4IMU imu;
Zumo32U4LineSensors ls;


#include "TurnSensor.h"

#define NUM_SENSORS 5
#define AVERAGE_ARRAY_SIZE 18 
#define SINUS_MAX 200
#define DIVISION_VALUE 18

unsigned int lineSensorValues[NUM_SENSORS];
int averageValues[AVERAGE_ARRAY_SIZE];
int total, averagePositionValue, averageDivided;
int16_t motor_1;
int16_t motor_2;
int32_t turnAngleComp;


void averagePosition(int16_t position) {
  static uint32_t counter, total = 0;
  total -= averageValues[counter % AVERAGE_ARRAY_SIZE];
  averageValues[counter % AVERAGE_ARRAY_SIZE] = position;
  total += averageValues[counter % AVERAGE_ARRAY_SIZE];
  averagePositionValue = total / AVERAGE_ARRAY_SIZE;

  counter++;
}

void calibrateSensors() {
  turnSensorSetup();
  delay(500);
  turnSensorReset();

  for (uint16_t i = 0; i < 90; i++) {
    motors.setSpeeds(100,-100);
    ls.calibrate();
    turnSensorUpdate();
    turnAngleComp = ((((int32_t)turnAngle >> 16) * 360) >> 16);
    
    display.gotoXY(0, 0);
    display.print(turnAngleComp);
    display.print(F("   "));

  }
  
  int16_t position = ls.readLine(lineSensorValues);
  do {
    turnSensorUpdate();
    position = ls.readLine(lineSensorValues);
    turnAngleComp = ((((int32_t)turnAngle >> 16) * 360) >> 16);
    
    display.gotoXY(0, 0);
    display.print(position);
    display.print(F("   "));

    motors.setSpeeds(100, -100);

  } while (turnAngleComp != 0 && (position >= 2050 && position <= 1950));

  motors.setSpeeds(0, 0);
  
}

void setup()
{

  for (int i = 0; i<AVERAGE_ARRAY_SIZE; i++) {
    averageValues[i] = 0;
  }

  ls.initFiveSensors();
  // turnSensorSetup();
  // delay(500);
  // turnSensorReset();

  motor_1 = 0;
  motor_2 = 0;

  // display.clear();
  // display.print(F("Try to"));
  // display.gotoXY(0, 1);
  // display.print(F("turn me!"));
  Serial.begin(9600);
}


void loop()
{
  static uint32_t i = 0;
  int16_t value;

  // ls.calibrate(); 
  int16_t position = ls.readLine(lineSensorValues);

  averagePosition(position);



  float angle = 0;

  if(buttonB.isPressed()) {
    calibrateSensors();
  }
  do {
    if (motor_1 > 0) {
      motor_1 -= 1;
      delay(1);
    }
    if (motor_2 > 0) {
      motor_2 -= 1;
      delay(1);
    }
    if (motor_1 < 0) {
      motor_2 += 1;
      delay(1);
    }
    if (motor_2 < 0) {
      motor_2 += 1;
      delay(1);
    }
    motors.setSpeeds(motor_1, motor_2);
  } while (motor_1 > 0 || motor_2 > 0 || motor_1 < 0 || motor_2 < 0);
  

  display.clear();
  display.gotoXY(0,0);
  display.print(averagePositionValue);
  delay(10);


  
  // Read the gyro to update turnAngle, the estimation of how far
  // the robot has turned, and turnRate, the estimation of how
  // fast it is turning.
  // turnSensorUpdate();

  // Calculate the motor turn speed using proportional and
  // derivative PID terms.  Here we are a using a proportional
  // constant of 56 and a derivative constant of 1/20.
  // int32_t turnSpeed = -(int32_t)turnAngle / (turnAngle1 / 56)
  //   - turnRate / 20;

  // Constrain our motor speeds to be between
  // -maxSpeed and maxSpeed.
  // turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);

  //  motors.setSpeeds(-turnSpeed, turnSpeed);
  if (buttonA.isPressed()) {
    delay(500);
    unsigned long ct = millis();

    do {
      turnSensorUpdate();
      angle = radians(i);
      value = (SINUS_MAX/2) * (1-cos(angle));
      position = ls.readLine(lineSensorValues);
      
      averagePosition(position);

      averageDivided = ((averagePositionValue-2000) / DIVISION_VALUE);


      int16_t absoluteDivided = abs(averageDivided);

      if (absoluteDivided == 0) {
        absoluteDivided = 1;
      }
      

      motor_1 = (( averageDivided ) + (value));
      motor_2 = -( averageDivided ) + (value);

      motor_1 = constrain(motor_1, -(value),value);
      motor_2 = constrain(motor_2, -(value),value);


      display.clear();
      display.gotoXY(0, 1);
      display.print(averageDivided);
      
      
      if (value < SINUS_MAX) {
        i++;
      }

      motors.setSpeeds(motor_1, motor_2);
    } while (millis() - ct <= 10000);

    i = 0;
  }
}
