#include <Arduino.h>
#include "imu.h"
#include "ahrs.h"
#include "complimentary.h"

Adafruit_LSM6DSOX sox;
Mahony ahrs_mahony{5.0F, 0.1F};
imu_T imu(sox, 0.1F, 40.0F * 2.0F * M_PI,
          40.0F * 2.0F * M_PI, 0.02F * 2.0F * M_PI);
ahrs_T sf(imu, ahrs_mahony);
complFilt_T cf{0.05F, 20.0F * 2.0F * M_PI, 0.1F};

float accelCal[3] = {0.0F,0.0F,0.0F};
float gyroCal[3] = {0.0F,0.0F,-0.0F};
//float gyroCal[3] = {-10.2F,0.2F,-2.2F};

void setup() {
    unsigned long baudrate = 500000;
    SetupImu(imu, baudrate);
    imu.SetCalibration(accelCal, gyroCal);
    ahrs_mahony.begin(10.0F);
}

void loop() {
    imu.Update();
    cf.Update(imu);
    sf.Update();

    Serial.print(micros() / 1000000.0F);
    Serial.print(",");
    PrintImu(imu);
    PrintAhrs(sf);
    PrintCf(cf);
    Serial.println();

    delay(90);
};