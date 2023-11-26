#pragma once
#define CALIBRATE_CONSTANT 32
#define WHEEL_DIAMETER 


const int32_t two_g = 0x80000000; // 100%
const int32_t one_g = two_g / 2; // 1%



Zumo32U4Encoders encoders;


int16_t accX;
int16_t accY;
int16_t accZ;

int16_t accOffsetX;
int16_t accOffsetY;
int16_t accOffsetZ;

int32_t velocity;

// int16_t velocity;
int16_t velocity_y;


uint16_t accLastUpdate;


void accReset(){
  accLastUpdate = micros();
  velocity = 0;          // reset velocity
}


void test() {
  Wire.begin();
  imu.init();
  imu.enableDefault();

  display.clear();
  display.print(F("Calibrate"));

  ledYellow(1);

  delay(500);

  int32_t totalAccY = 0;
  int32_t totalAccX = 0;
  int32_t totalAccZ = 0;
  // unsigned long t_before = millis();


  for (uint16_t i = 0; i < CALIBRATE_CONSTANT; i++) {
    while (!imu.accDataReady()) {}
    // Serial.println(i);
    imu.read();
    totalAccX += imu.a.x;
    totalAccY += imu.a.y;
    totalAccZ += imu.a.z;
  }
  // unsigned long t_after = millis() - t_before;
  
  // t_after = 2260

  // ca. 13 oppdateringer/s
  // ca. hver 70ms




  
  ledYellow(0);

  accOffsetX = totalAccX / CALIBRATE_CONSTANT;
  accOffsetY = totalAccY / CALIBRATE_CONSTANT;
  accOffsetZ = totalAccZ / CALIBRATE_CONSTANT;

  display.clear();

  accReset();

  unsigned long time_before = millis();

  do
  {
    

    // int32_t magnitudeSquared = (int32_t)acc_x * acc_x + (int32_t)acc_y * acc_y;
    //if(imu.accDataReady()) {
      imu.read();
      accX = imu.a.x - accOffsetX;
      accY = imu.a.y - accOffsetY;
      accY = imu.a.z - accOffsetZ;
      // accY = ((int32_t)accY*1962)/65535; // 2 (g) * 98 cm/s^2
      accX = ((int32_t)accX*19620)/65535; // 2 (g) * 98 cm/s^2


      uint16_t m = micros();
      uint16_t dt = m - accLastUpdate;
      accLastUpdate = m;
      int32_t d = (int32_t)accX * dt;
      velocity += ((float)d) / 1000000.0f;
  
      // uint16_t m = micros();
      // uint16_t dt = m - accLastUpdate;
      // accLastUpdate = m;
      // int32_t d = (int32_t)accY*dt;
      // velocity += (int64_t)d;
      // 1g = 9.81m/s,  * 14  => ms -> s
      // (4/2^16 g/digit) * (1/1000 ms/s) * 9.81 g * 1/1000 s/ms
      int32_t convertVelocity = (((int32_t)velocity >> 16));
      display.gotoXY(0, 0);
      display.print(velocity);
      display.print(F("       "));
      display.gotoXY(0, 1);
      display.print(d);
      display.print(F("       "));
    //}
    
    // (0.07 dps/digit) * (1/1000000 s/us) * (2^29/45 unit/degree)

    // +/- 2g max
    // 75.81 × 12 ≈ 909.7 CPR.
    // A value of 16384 corresponds to approximately 1 g.
    // acc = rate = m/s^2
    // begrunn avgjørelser



    // static uint8_t lastDisplayTime;
    
    // if ((uint8_t)(millis() - lastDisplayTime) > 100)
    // {
    //   lastDisplayTime = millis();
    //   display.gotoXY(0, 0);
    //   display.print(convertVelocity);
    //   display.print(F("       "));
    //   display.gotoXY(0, 1);
    //   display.print("N/A");
    //   display.print(F("       "));
    // }

  } while (millis() - time_before <= 100000);
  






}



