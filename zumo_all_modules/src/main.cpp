#include <Wire.h>
#include <Zumo32U4.h>

Zumo32U4IMU imu;
Zumo32U4OLED display;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4Encoders encoders;

struct Highest
{
  int16_t firstRun;
  int16_t secondRun;
  int16_t median = (firstRun + secondRun) / 2;
};
Highest hr;


int16_t accX;
int32_t velocity;
uint16_t accLastUpdate;
int32_t highestValue = 0;



void accReset(){
  accLastUpdate = micros();
  velocity = 0;          // reset velocity
}

char report[120];

char sb01[16], sb02[16], sb03[16];


// inspired from example code
void update() {
  imu.read();
  accX = imu.a.x;
  accX = ((int32_t)accX*19620)/65535; // 2 (g) * 981 cm/s^2 / 2^16 - 1

  uint16_t m = micros();
  uint16_t dt = m - accLastUpdate;
  accLastUpdate = m;
  int32_t d = (int32_t)accX * dt;
  velocity += d / 1000000;

  display.gotoXY(0, 0);
  display.print(velocity);
  display.print(F("       "));
}

void getHighestVelocity() {
  if (velocity > highestValue){
    highestValue = velocity;
  }
}

void displayView(char topScreen[16], char bottomScreen[16]) {
  display.gotoXY(0, 0);
  display.print(topScreen);
  display.print(F("       "));
  display.gotoXY(0,1);
  display.print(bottomScreen);
  display.print(F("       "));
}

void displayView(char topScreen[16], char middleScreen[16], char bottomScreen[16]) {
  display.gotoXY(0, 0);
  display.print(topScreen);
  display.print(F("       "));
  display.gotoXY(0,1);
  display.print(middleScreen);
  display.print(F("       "));
  display.gotoXY(0,2);
  display.print(bottomScreen);
  display.print(F("       "));
} 

void setup()
{
  Wire.begin();



  accReset();
  imu.enableDefault();
  display.setLayout11x4();
  display.clear();
  buttonA.waitForButton();

}

void loop()
{
  static int16_t previousCountsLeft = 0;
  static uint16_t lastEncoderUpdate = 0; 
  int16_t encoderLeft = encoders.getCountsLeft();
  int16_t encoderCount;
  static int32_t d;

  // encoder resolution: 12 / revolution
  // 909.7 cpr
  // diameter: 39mm
  // motors.setSpeeds(400,400);

  if (encoderLeft != previousCountsLeft) {
    uint16_t now = micros();
    uint16_t dt = now - lastEncoderUpdate;
    lastEncoderUpdate = now;
    encoderCount = encoderLeft - previousCountsLeft;
    
    // encoder resolution: 12 / revolution
    // 909.7 cpr
    // diameter: 39mm
    // (encoderCount*39*pi*100000000)/(dt*9097) - skalert for høyere nøyaktighet og unngå floating point operations (mm/s)
    int16_t encoderSpeed = (int64_t)encoderCount * 1225221134 / ((uint64_t)dt * 9097);

    previousCountsLeft = encoderLeft;
    
    // pi * 39mm * encoderLeft * 10 / x cpr (mm)
    d += (encoderCount * 1225)/9097;

    // print til display
    char encoderRPM[10], distance[10];
    snprintf_P(encoderRPM, sizeof(encoderRPM), PSTR("0,%2d m/s"), encoderSpeed);
    snprintf_P(distance, sizeof(distance), PSTR("%2d mm"), d);
    displayView(encoderRPM, distance);

  }

  //update();
  
  if (buttonA.isPressed()) {
    unsigned long now = millis();
    delay(1000);
    display.clear();
    update();
    do {
      update();
      motors.setSpeeds(120,120);
      getHighestVelocity();
      delayMicroseconds(13);
      hr.firstRun = velocity;
    } while (millis() - now < 2000); 
      motors.setSpeeds(0,0);
      delay(100);
      accReset();
      update();
      delay(1000);
      unsigned long now2 = millis();
    do {
      update();
      motors.setSpeeds(320,320);
      getHighestVelocity();
      delayMicroseconds(13);
      hr.secondRun = velocity;
    } while (millis() - now2 < 2000); 
    accReset();
    motors.setSpeeds(0,0);
    unsigned long now3 = millis();
    display.clear();
    do {
      snprintf_P(sb01, sizeof(sb01), PSTR("f: 0,%2d m/s"), hr.firstRun);
      snprintf_P(sb02, sizeof(sb02), PSTR("s: 0,%2d m/s"), hr.secondRun);
      snprintf_P(sb03, sizeof(sb03), PSTR("m: 0,%2d m/s"), (hr.firstRun + hr.secondRun) / 2);
      displayView(sb01, sb02,sb03);
    } while (millis() - now3 < 100000);
  }

  static unsigned long now = 0;
  static unsigned long interval = 100;

  //delay(100);

  if(millis() - now < interval) {
    now = millis();
    Serial.println(velocity);
  }
}