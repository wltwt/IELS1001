#include <Wire.h>
#include <Zumo32U4.h>
#include <EEPROM.h>

// kilder:
// https://www.geeksforgeeks.org/when-do-we-use-initializer-list-in-c/
// https://forum.arduino.cc/t/c-object-instantiation-in-setup/337684
// https://forum.arduino.cc/t/how-do-i-use-enum/70307
// https://docs.arduino.cc/learn/programming/eeprom-guide


enum State {CHARGING, DISCHARGING}; 
enum EEPROMaddress {bLevel = 0x00, bCycles = 0x01, bHealth = 0x02};


int IR_CHARGING = 0;  // temp

Zumo32U4IMU imu;
Zumo32U4OLED display;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4Encoders encoders;


struct Battery {
  State state = DISCHARGING;
  int16_t battery_level;
  uint16_t battery_cycles;
  int16_t battery_health;

  Battery(int bCycles, int bHealth) : battery_cycles(bCycles), battery_health(bHealth) {
    checkCurrentState();
  }

  Battery() : battery_cycles() {
    checkCurrentState();
    getEEPROM();
  }

  void checkCurrentState() {
    if (IR_CHARGING) {
      state = CHARGING;
    }
  }

  // setter som private, vil unngå unødvendig tilgang til EEPROM
  private:
    void getEEPROM() {
      EEPROM.get(bLevel, battery_cycles);
    }
};

void displayView(char topScreen[16], char bottomScreen[16]) {
  display.gotoXY(0, 0);
  display.print(topScreen);
  display.print(F("       "));
  display.gotoXY(0,1);
  display.print(bottomScreen);
  display.print(F("       "));
}

struct Position {
  // ordne slik at alt som har med posisjon og hastighet skjer her
  int32_t distance;

  struct Encoder {
    int16_t left, right;                                       // encoder count
    int16_t countL = 0, countR = 0, pCountL = 0, pCountR = 0;
    uint16_t lastUpdate = 0;
    Encoder() : left(), right() {
      left = encoders.getCountsAndResetLeft();
      right = encoders.getCountsAndResetRight();
    }
  };

  Encoder *enc = NULL;

  Position() : enc() {
    enc = new Encoder();
    distance = 0;
  }

  void update() {
    enc->left = encoders.getCountsLeft();
    enc->right = encoders.getCountsRight();
  }

  void getSpeed() {
    
    update();

    // Serial.println(enc->count);

    if (enc->left != enc->pCountL) { //|| enc->right != enc->pCountR) {
      uint16_t now = micros();
      uint16_t dt = now - enc->lastUpdate;
      enc->lastUpdate = now;
      enc->countL = enc->left - enc->pCountL;

      
      // encoder resolution: 12 / revolution
      // 909.7 cpr
      // diameter: 39mm
      // (encoderCount*39*pi*100000000)/(dt*9097) - skalert for høyere nøyaktighet og unngå floating point operations (mm/s)
      int16_t encoderSpeed = (int64_t)enc->countL * 1225221134 / ((int64_t)dt * 9097);

      enc->pCountL = enc->left;
      
      // pi * 39mm * encoderLeft * 10 / x cpr (mm)
      distance += (enc->countL * 1225)/9097;

      char encoderRPM[10], distance_thing[10];
      snprintf_P(encoderRPM, sizeof(encoderRPM), PSTR("0,%2d m/s"), encoderSpeed);
      snprintf_P(distance_thing, sizeof(distance_thing), PSTR("%2d mm"), distance);
      displayView(encoderRPM, distance_thing);

    }
  }
};



Battery *battery = NULL;  // oppretter kun pointer
Position *pos = NULL;

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

int storeEEPROMtest = 0;

void accReset(){
  accLastUpdate = micros();
  velocity = 0;               // reset velocity
}

char report[120];

char sb01[16], sb02[16], sb03[16];


// inspired from example code
// void update() {
//   imu.read();
//   accX = imu.a.x;
//   accX = ((int32_t)accX*19620)/65535; // 2 (g) * 981 cm/s^2 / 2^16 - 1

//   uint16_t m = micros();
//   uint16_t dt = m - accLastUpdate;
//   accLastUpdate = m;
//   int32_t d = (int32_t)accX * dt;
//   velocity += d / 1000000;

//   display.gotoXY(0, 0);
//   display.print(velocity);
//   display.print(F("       "));
// }

void getHighestVelocity() {
  if (velocity > highestValue){
    highestValue = velocity;
  }
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
  Serial.begin(9600);
  // EEPROM.put(0x00, 17823);

  EEPROM.get(0x00, storeEEPROMtest);

  battery = new Battery();
  pos = new Position();


  accReset();
  imu.enableDefault();
  display.setLayout11x4();
  display.clear();
  buttonA.waitForButton();

}

void loop()
{

  // static int16_t previousCountsLeft = 0;
  // static uint16_t lastEncoderUpdate = 0; 
  // int16_t encoderLeft = encoders.getCountsLeft();
  // int16_t encoderCount;
  // static int32_t d;
  
  // Serial.println(battery->battery_health);

  if (buttonA.isPressed()) {
    unsigned long now = millis();
    do {
      motors.setSpeeds(150,150);

      pos->getSpeed();

    
    
    } while (millis() - now < 2000); 
    motors.setSpeeds(0,0);
  }



  delay(1);

  // encoder resolution: 12 / revolution
  // 909.7 cpr
  // diameter: 39mm
  // motors.setSpeeds(400,400);

  // if (encoderLeft != previousCountsLeft) {
  //   uint16_t now = micros();
  //   uint16_t dt = now - lastEncoderUpdate;
  //   lastEncoderUpdate = now;
  //   encoderCount = encoderLeft - previousCountsLeft;
    
  //   // encoder resolution: 12 / revolution
  //   // 909.7 cpr
  //   // diameter: 39mm
  //   // (encoderCount*39*pi*100000000)/(dt*9097) - skalert for høyere nøyaktighet og unngå floating point operations (mm/s)
  //   int16_t encoderSpeed = (int64_t)encoderCount * 1225221134 / ((uint64_t)dt * 9097);

  //   previousCountsLeft = encoderLeft;
    
  //   // pi * 39mm * encoderLeft * 10 / x cpr (mm)
  //   d += (encoderCount * 1225)/9097;

  //   // print til display
  //   char encoderRPM[10], distance[10];
  //   snprintf_P(encoderRPM, sizeof(encoderRPM), PSTR("0,%2d m/s"), encoderSpeed);
  //   snprintf_P(distance, sizeof(distance), PSTR("%2d mm"), d);
  //   displayView(encoderRPM, distance);
  // }






  if (battery->state == CHARGING) {
    motors.setSpeeds(0,0);
  }
  //update();
  
  
    // if (buttonB.isPressed()) {
    //   unsigned long now = millis();
    //   delay(1000);
    //   display.clear();
    //   update();
    //   do {
    //     update();
    //     motors.setSpeeds(120,120);
    //     getHighestVelocity();
    //     delayMicroseconds(13);
    //     hr.firstRun = velocity;
    //   } while (millis() - now < 2000); 
    //     motors.setSpeeds(0,0);
    //     delay(100);
    //     accReset();
    //     update();
    //     delay(1000);
    //     unsigned long now2 = millis();
    //   do {
    //     update();
    //     motors.setSpeeds(320,320);
    //     getHighestVelocity();
    //     delayMicroseconds(13);
    //     hr.secondRun = velocity;
    //   } while (millis() - now2 < 2000); 
    //   accReset();
    //   motors.setSpeeds(0,0);
    //   unsigned long now3 = millis();
    //   display.clear();
    //   do {
    //     snprintf_P(sb01, sizeof(sb01), PSTR("f: 0,%2d m/s"), hr.firstRun);
    //     snprintf_P(sb02, sizeof(sb02), PSTR("s: 0,%2d m/s"), hr.secondRun);
    //     snprintf_P(sb03, sizeof(sb03), PSTR("m: 0,%2d m/s"), (hr.firstRun + hr.secondRun) / 2);
    //     displayView(sb01, sb02,sb03);
    //   } while (millis() - now3 < 100000);
    // }

  static unsigned long now = 0;
  static unsigned long interval = 100;

  //delay(100);

  if(millis() - now < interval) {
    now = millis();
    Serial.println(velocity);
  }
}