/* This example shows how to:

- Measure the voltage of the Zumo's batteries.
- Detect whether USB power is present.

The results are printed to the display and also to the serial
monitor.

The battery voltage can only be read when the power switch is in
the "On" position.  If the power switch is off, the voltage
reading displayed by this demo will be low, but it might not be
zero because the Zumo 32U4 has capacitors that take a while to
discharge. */

#include <Wire.h>
#include <Zumo32U4.h>

// Change next line to this if you are using the older Zumo 32U4
// with a black and green LCD display:
// Zumo32U4LCD display;
Zumo32U4OLED display;

Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;

Zumo32U4Encoders encoders;
Zumo32U4Buzzer buzzer;
// Zumo32U4Motors motors;
// Zumo32U4ButtonA buttonA;
Zumo32U4ButtonC buttonC;

// Change next line to this if you are using the older Zumo 32U4
// with a black and green LCD display:
// Zumo32U4LCD display;
// Zumo32U4OLED display;

const char encoderErrorLeft[] PROGMEM = "!<c2";
const char encoderErrorRight[] PROGMEM = "!<e2";

char report[80];

char report[120];


Zumo32U4IMU imu;



void setup()
{

  Wire.begin();

  if (!imu.init())
  {
    // Failed to detect the compass.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to initialize IMU sensors."));
      delay(100);
    }
  }

  imu.enableDefault();

  // Wait for the user to press button A.
  buttonA.waitForButton();

  // Delay so that the robot does not move away while the user is
  // still touching it.
  delay(1000);
  

}

void loop()
{
  bool usbPower = usbPowerPresent();

  uint16_t batteryLevel = readBatteryMillivolts();

  // Print the results to the display.
  // display.clear();
  // display.print(F("B="));
  // display.print(batteryLevel);
  // display.print(F("mV   "));
  // display.gotoXY(0, 1);
  // display.print(F("USB="));
  // display.print(usbPower ? 'Y' : 'N');

  // // Print the results to the serial monitor.
  // Serial.print(F("USB="));
  // Serial.print(usbPower ? 'Y' : 'N');
  // Serial.print(F(" B="));
  // Serial.print(batteryLevel);
  // Serial.println(F(" mV"));

  static uint8_t lastDisplayTime;
  static uint8_t displayErrorLeftCountdown = 0;
  static uint8_t displayErrorRightCountdown = 0;

  if ((uint8_t)(millis() - lastDisplayTime) >= 100)
  {
    lastDisplayTime = millis();

    int16_t countsLeft = encoders.getCountsLeft();
    int16_t countsRight = encoders.getCountsRight();

    bool errorLeft = encoders.checkErrorLeft();
    bool errorRight = encoders.checkErrorRight();

    if (errorLeft)
    {
      // An error occurred on the left encoder channel.
      // Show it on the display for the next 10 iterations and
      // also beep.
      displayErrorLeftCountdown = 10;
      buzzer.playFromProgramSpace(encoderErrorLeft);
    }

    if (errorRight)
    {
      // An error occurred on the right encoder channel.
      // Show it on the display for the next 10 iterations and
      // also beep.
      displayErrorRightCountdown = 10;
      buzzer.playFromProgramSpace(encoderErrorRight);
    }

    // Update the display with encoder counts and error info.
    display.clear();
    display.print(countsLeft);
    display.gotoXY(0, 1);
    display.print(countsRight);

    if (displayErrorLeftCountdown)
    {
      // Show an exclamation point on the first line to
      // indicate an error from the left encoder.
      display.gotoXY(7, 0);
      display.print('!');
      displayErrorLeftCountdown--;
    }

    if (displayErrorRightCountdown)
    {
      // Show an exclamation point on the second line to
      // indicate an error from the left encoder.
      display.gotoXY(7, 1);
      display.print('!');
      displayErrorRightCountdown--;
    }

    // Send the information to the serial monitor also.
    snprintf_P(report, sizeof(report),
        PSTR("%6d %6d %1d %1d"),
        countsLeft, countsRight, errorLeft, errorRight);
    Serial.println(report);
  }

  if (buttonA.isPressed())
  {
    static int speed;
    for (speed = 0; speed <= 400; speed++) {
      motors.setSpeeds(speed, speed);
      delay(2);
    }
    for (speed = speed; speed >= 0; speed--){
      motors.setSpeeds(speed,speed);
      delay(2);
    }
  }
  else if (buttonC.isPressed())
  {
    for (int speed = 0; speed >= -400; speed--) {
      motors.setSpeeds(speed, speed);
      delay(2);
    }
    for (int speed = -400; speed <= 0; speed++) {
      motors.setSpeeds(speed, speed);
      delay(2);
    }
  }
  else
  {
    motors.setSpeeds(0, 0);
  }

  delay(500);
}