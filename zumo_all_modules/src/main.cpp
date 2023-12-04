#include <Wire.h>
#include <Zumo32U4.h>
#include <EEPROM.h>

// kilder:
// https://www.geeksforgeeks.org/when-do-we-use-initializer-list-in-c/
// https://forum.arduino.cc/t/c-object-instantiation-in-setup/337684
// https://forum.arduino.cc/t/how-do-i-use-enum/70307
// https://docs.arduino.cc/learn/programming/eeprom-guide

// encoder resolution: 12 / revolution
// 909.7 cpr
// diameter: 39mm

#define AVERAGE_INTERVAL 2000               // ms
#define FULL_CHARGE 100
#define PERCENT_OF_MAX 200  

enum BatteryState {
  STATE_CHARGING, 
  STATE_DISCHARGING
};

enum EEPROMaddress {
  bLevel = 0x00,
  bCycles = 0x01, 
  bHealth = 0x02
};


int IR_CHARGING = 0;  // temp

Zumo32U4IMU imu;
Zumo32U4OLED display;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4ButtonB buttonB;
Zumo32U4Encoders encoders;

void displayView(char topScreen[16], char bottomScreen[16]) {
  display.gotoXY(0, 0);
  display.print(topScreen);
  display.print(F("       "));
  display.gotoXY(0,1);
  display.print(bottomScreen);
  display.print(F("       "));
}


struct Battery {
  BatteryState state = STATE_DISCHARGING;
  int16_t CHARGE_CONSTANT = 10;
  int16_t battery_level;
  uint16_t battery_cycles;
  int16_t battery_health;
  int16_t discharge_factor;

  Battery(int bCycles, int bHealth) : battery_cycles(bCycles), battery_health(bHealth) {
    checkCurrentState();
  }

  Battery() : battery_cycles() {
    checkCurrentState();
    getEEPROM();
  }

  void discharging(int16_t distance, int16_t avgVelocity) {
    Serial.print(">batterycalculating:");
    Serial.println(distance + avgVelocity);
    Serial.print(">batterycalculating*10:");
    Serial.println( (distance + avgVelocity) * 10);
    Serial.print(">batterycalculating/1000:");
    Serial.println((distance + avgVelocity)/5000);
    
    battery_level = 100 - ((distance + avgVelocity)*10) / 5000;
    
  }

  void charging() {
    if (battery_level <= FULL_CHARGE) {
      battery_level += CHARGE_CONSTANT;
    }
  }

  void checkCurrentState() {
    if (IR_CHARGING) {
      state = STATE_CHARGING;
    }
  }

  BatteryState getCurrentState() {
    return state;
  }

  // setter som private, vil unngå unødvendig tilgang til EEPROM
  private:
    void getEEPROM() {
      // EEPROM.get(bLevel, battery_cycles);
      battery_level = FULL_CHARGE;
    }
};


Battery *battery = NULL;  // oppretter kun pointer



struct Position {
  // ordne slik at alt som har med posisjon og hastighet skjer her
  int32_t distance;
  int16_t speed, maxSpeed;
  int16_t previousAverageDistance = 0, averageVelocity = 0, timeBelow = 0, timeAboveTotal = 0;
  int32_t timeAbove = 0, totalTimeAbove = 0;
  unsigned long prev, speedTimer, belowTimer, aboveTimer;

  struct Encoder {
    int16_t left, right;                                       // encoder count
    int16_t countL = 0, countR = 0, pCountL = 0, pCountR = 0;
    uint16_t lastUpdate = 0;
    Encoder() : left(), right() {
      left = encoders.getCountsAndResetLeft();
      right = encoders.getCountsAndResetRight();
    }
  };

  Encoder enc;

  Position() : enc() {
    distance = 0;
    maxSpeed = 0;
  }

  private:
    void update() {
      enc.left = encoders.getCountsLeft();
      enc.right = encoders.getCountsRight();
    }

    void calculateSpeed() {    
      update();

      if ((enc.left != enc.pCountL) || (enc.right != enc.pCountR)) {
        uint16_t now = micros();
        uint16_t dt = now - enc.lastUpdate;
        enc.lastUpdate = now;
        enc.countL = enc.left - enc.pCountL;
        enc.countR = enc.right - enc.pCountR;

        // (encoderCount*39*pi*100000000)/(dt*9097) - skalert for høyere nøyaktighet og unngå floating point operations (mm/s)
        int16_t speedL = (int64_t)enc.countL * 1225221134 / ((int64_t)dt * 9097);
        int16_t speedR = (int64_t)enc.countR * 1225221134 / ((int64_t)dt * 9097);

        enc.pCountL = enc.left;
        enc.pCountR = enc.right;

        int16_t speed = (speedL + speedR) / 2;

        getSpeed(speed);
        getMaxSpeed(speed);
        
        // Serial.print(">lastUpdate:");
        // Serial.println(micros() - enc.lastUpdate);
        // Serial.print(">previousAverage:");
        // Serial.println(previousAverageDistance);

        // pi * 39mm * encoderLeft * 10 / x cpr (mm)
        distance += (((enc.countL+enc.countR)/2) * 1225)/9097;
        battery->discharging(distance, averageVelocity);
        Serial.print(">battery_level:");
        Serial.println(battery->battery_level);

        char encoderRPM[10], distance_thing[10];
        snprintf_P(encoderRPM, sizeof(encoderRPM), PSTR("0,%2d m/s"), speed);
        snprintf_P(distance_thing, sizeof(distance_thing), PSTR("%2d mm"), distance);
        displayView(encoderRPM, distance_thing);
      } else {
        speed = 0;
      }
      generateStats();
    }

    void getSpeed(int16_t s) {
      speed = s;
    }

    void getMaxSpeed(int16_t v) {
      if (v > maxSpeed) {
        maxSpeed = v;
      }
    }

    void generateStats() {
      uint16_t now = micros();
      Serial.print(">speed:");
      Serial.println(speed);

      // bilen står stille
      if (now - enc.lastUpdate > 5000){
        prev = 0;
        averageVelocity = 0;
        maxSpeed = 0;
        previousAverageDistance = distance;
        
        // bilen beveger seg
      } else {
        unsigned long now = millis();
        if (now - prev > AVERAGE_INTERVAL) {
          prev = now;
          speedTimer = now;
          aboveTimer = 0;
          averageVelocity = (distance - previousAverageDistance) / 2;
          previousAverageDistance = distance;
          totalTimeAbove = 0;
        }
        
        getMaxSpeed(speed);

        if (speed > PERCENT_OF_MAX) {
          if (now - aboveTimer > AVERAGE_INTERVAL) {
            aboveTimer = now;
            timeAbove = now - speedTimer;
            totalTimeAbove += timeBelow - timeAbove;
            belowTimer = 0;
          }
        } else {
          if (now - belowTimer > AVERAGE_INTERVAL) {
            belowTimer = now;
            timeBelow = now - speedTimer;
            totalTimeAbove += timeBelow - timeAbove;
            aboveTimer = 0;
          }
        }

        // Serial.print(">TimeAboveTotalStart:");
        // Serial.println(timeAboveTotal);
        // Serial.print(">Totaltimeabove:");
        // Serial.println(totalTimeAbove);
        // Serial.print(">abovetimer:");
        // Serial.println(timeAbove);
        // Serial.print(">belowtimer:");
        // Serial.println(timeBelow);
      }
    }

  public:
    void updatePosition() {
      calculateSpeed();
    }

};

Position *pos = NULL;


struct Account {
  int16_t ID;
  int16_t account_saldo;


};


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
// tillegg?
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

  char speeds1[10], speeds2[10];

  switch (battery->state) {
    case STATE_CHARGING:
      motors.setSpeeds(0,0);
      break;
    case STATE_DISCHARGING:
      // continue as normal?
      break;
  }

  if (buttonA.isPressed()) {
    int a = 10;
    int *t;
    t = &a;
    int *b = t;

    int *c[5] = {t,b};

    // Serial.println((unsigned long)*c[0]);
    

    // Serial.println((unsigned long)*t);
    // Serial.println((unsigned long)*b);

    // Serial.println((uint16_t)&pos->enc);

    unsigned long now = millis();
    hr.firstRun = 0;  
    do {
      pos->updatePosition();

      motors.setSpeeds(150,150);  
      if (pos->speed > hr.firstRun) {
        hr.firstRun = pos->speed;
      }
      // Serial.print(">hr1:");
      // Serial.println(pos->speed);
    
    } while (millis() - now < 1500); 

    now = millis();

    do {
      pos->updatePosition();
      motors.setSpeeds(100,100);

    } while (millis() - now < 2000);


    
    
    now = millis();
    hr.secondRun = 0;
    do {
      pos->updatePosition();

      motors.setSpeeds(300,300);  

      if (pos->speed > hr.secondRun) {
        hr.secondRun = pos->speed;
      }
      // Serial.print(">hr2:");
      // Serial.println(pos->speed);

    } while (millis() - now < 2000); 
    snprintf_P(speeds1, sizeof(speeds1), PSTR("%2d speed1"), hr.firstRun);
    snprintf_P(speeds2, sizeof(speeds2), PSTR("%2d speed2"), hr.secondRun);
    // Serial.println(hr.firstRun);
    // Serial.println(hr.secondRun);


    
    motors.setSpeeds(0,0);
  }

  displayView(speeds1, speeds2);

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






  if (battery->state == STATE_CHARGING) {
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