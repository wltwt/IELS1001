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
// #include <IRremote.h>
// #include <Zumo32u4IRsender.h>
#include "RemoteConstants.h"
#include "RemoteDecoder.h"

// #define DEVICE_ID 0x01
// #define DIRECTION RIGHT_IR
// #define IR_RECEIVE_PIN 20

// #    if defined(ARDUINO_AVR_PROMICRO) // Sparkfun Pro Micro is __AVR_ATmega32U4__ but has different external circuit
// // We have no built in LED at pin 13 -> reuse RX LED
// #undef LED_BUILTIN
// #define LED_BUILTIN         LED_BUILTIN_RX
// #    endif



// Zumo32U4IRsender ZumoIrSender(DEVICE_ID, DIRECTION); 


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
Zumo32U4ButtonC buttonC;

Zumo32U4Motors motors;
Zumo32U4IMU imu;
Zumo32U4LineSensors ls;

RemoteDecoder decoder;


#include "TurnSensor.h"
#include "Speed.h"

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

Zumo32U4Buzzer buzzer;
const uint16_t messageTimeoutMs = 115;


bool messageActive = false;

uint16_t lastMessageTimeMs = 0;




// Start running the new command.
void processRemoteCommand(uint8_t command)
{
  switch(command)
  {
  case remoteUp:
    display.print(F("up"));
    motors.setSpeeds(400, 400);
    break;

  case remoteDown:
    display.print(F("down"));
    motors.setSpeeds(-400, -400);
    break;

  case remoteLeft:
    display.print(F("left"));
    motors.setSpeeds(-250, 250);
    break;

  case remoteRight:
    display.print(F("right"));
    motors.setSpeeds(250, -250);
    break;

  case remoteStopMode:
    display.print(F("stop"));
    break;

  case remoteEnterSave:
    display.print(F("enter"));
    break;

  case remoteVolMinus:
    display.print(F("vol-"));
    break;

  case remoteVolPlus:
    display.print(F("vol+"));
    break;

  case remotePlayPause:
    display.print(F("play"));
    break;

  case remoteSetup:
    display.print(F("setup"));
    break;

  case remoteBack:
    display.print(F("back"));
    break;

  case remote0:
    display.print(F("0"));
    break;

  case remote1:
    display.print(F("1"));
    buzzer.playNote(NOTE_C(4), 200, 15);
    break;

  case remote2:
    display.print(F("2"));
    buzzer.playNote(NOTE_D(4), 200, 15);
    break;

  case remote3:
    display.print(F("3"));
    buzzer.playNote(NOTE_E(4), 200, 15);
    break;

  case remote4:
    display.print(F("4"));
    break;

  case remote5:
    display.print(F("5"));
    break;

  case remote6:
    display.print(F("6"));
    break;

  case remote7:
    display.print(F("7"));
    break;

  case remote8:
    display.print(F("8"));
    break;

  case remote9:
    display.print(F("9"));
    break;
  }
}

void stopCurrentCommand()
{
  motors.setSpeeds(0, 0);
  buzzer.stopPlaying();
}

void processRemoteMessage(const uint8_t * message)
{
  // Print the raw message on the first line of the display, in hex.
  // The first two bytes are usually an address, and the third
  // byte is usually a command.  The last byte is supposed to be
  // the bitwise inverse of the third byte, and if that is the
  // case, then we don't print it.
  display.clear();
  char buffer[9];
  if (message[2] + message[3] == 0xFF)
  {
    sprintf(buffer, "%02X%02X %02X ",
      message[0], message[1], message[2]);
  }
  else
  {
    sprintf(buffer, "%02X%02X%02X%02X",
      message[0], message[1], message[2], message[3]);
  }
  display.print(buffer);

  // Go to the next line of the display.
  display.gotoXY(0, 1);

  // Make sure the address matches what we expect.
  if (message[0] != remoteAddressByte0 ||
    message[1] != remoteAddressByte1)
  {
    display.print(F("bad addr"));
    return;
  }

  // Make sure that the last byte is the logical inverse of the
  // command byte.
  if (message[2] + message[3] != 0xFF)
  {
    display.print(F("bad cmd"));
    return;
  }

  stopCurrentCommand();
  processRemoteCommand(message[2]);
}


void processRemoteEvents()
{
  if (decoder.getAndResetMessageFlag())
  {
    // The remote decoder received a new message, so record what
    // time it was received and process it.
    lastMessageTimeMs = millis();
    messageActive = true;
    processRemoteMessage(decoder.getMessage());
  }

  if (decoder.getAndResetRepeatFlag())
  {
    // The remote decoder receiver a "repeat" command, which is
    // sent about every 109 ms while the button is being held
    // down.  It contains no data.  We record what time the
    // repeat command was received so we can know that the
    // current message is still active.
    lastMessageTimeMs = millis();
  }
}




// Stops the current remote control command.  This is called when
// a new command is received or if the current command has
// expired.

void setup()
{
  // Serial.begin(9600);
  decoder.init();
  display.clear();
  display.print(F("Waiting"));
  // test();

  // for (int i = 0; i<AVERAGE_ARRAY_SIZE; i++) {
  //   averageValues[i] = 0;
  // }

  // ls.initFiveSensors();
  // turnSensorSetup();
  // delay(500);
  // turnSensorReset();
  // Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));
  // Serial.print(F("Ready to receive IR signals of protocols: "));
  // motor_1 = 0;
  // motor_2 = 0;
  // IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  // IrReceiver.printActiveIRProtocols(&Serial);

  

  // display.clear();
  // display.print(F("Try to"));
  // display.gotoXY(0, 1);
  // display.print(F("turn me!"));
}


void loop()
{
  static uint32_t i = 0;
  int16_t value;

  // Serial.println("Test");

  // if(IrReceiver.decode()) {
  //   IrReceiver.printIRResultShort(&Serial);
  //   IrReceiver.printIRSendUsage(&Serial);
  //   Serial.println("Test");

  //   IrReceiver.resume();
  //   if (IrReceiver.decodedIRData.command == 0x10) {
  //       // do something
  //       Serial.println("0x10");
  //   } else if (IrReceiver.decodedIRData.command == 0x11) {
  //       // do something else
  //       Serial.println("0x11");
  //   }
  // }

  decoder.service();
    
  ledYellow(messageActive);

  ledRed(decoder.criticalTime());

  if (decoder.criticalTime()) {
  } else {
    processRemoteEvents();
  }
  
  if (messageActive && (uint16_t)(millis() - lastMessageTimeMs) > messageTimeoutMs)
  {
    messageActive = false;
    stopCurrentCommand();
  }

  
  delay(10);


  // ls.calibrate(); 
  // int16_t position = ls.readLine(lineSensorValues);

  // averagePosition(position);



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

  if(buttonC.isPressed()) {
    test();


  }
  

  // display.clear();
  // display.gotoXY(0,0);
  // display.print(averagePositionValue);


  
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
      // position = ls.readLine(lineSensorValues);
      
      // averagePosition(position);

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

