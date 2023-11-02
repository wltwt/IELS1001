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
  bool usbPower = usbPowerPresent();

  uint16_t batteryLevel = readBatteryMillivolts();

  // Print the results to the display.
  display.clear();
  display.print(F("B="));
  display.print(batteryLevel);
  display.print(F("mV   "));
  display.gotoXY(0, 1);
  display.print(F("USB="));
  display.print(usbPower ? 'Y' : 'N');

  // Print the results to the serial monitor.
  Serial.print(F("USB="));
  Serial.print(usbPower ? 'Y' : 'N');
  Serial.print(F(" B="));
  Serial.print(batteryLevel);
  Serial.println(F(" mV"));

  delay(200);
}