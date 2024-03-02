#include <Arduino.h>
#include "imu.h"
#include "ahrs.h"
#include "complimentary.h"

Adafruit_LSM6DSOX sox;
Mahony ahrs_mahony{5.0F, 0.1F};
imu_T imu(sox, 0.1F, 40.0F * 2.0F * M_PI,
          40.0F * 2.0F * M_PI, 0.02F * 2.0F * M_PI);
ahrs_T ahrs(imu, ahrs_mahony);
complFilt_T cf{0.05F, 20.0F * 2.0F * M_PI, 0.1F};

float accelCal[3] = {0.0F,0.0F,0.0F};
float gyroCal[3] = {0.0F,0.0F,-0.0F};

bool visualise = true;

void setup() {
    unsigned long baudrate = 500000;
    SetupImu(imu, baudrate);
    imu.SetCalibration(accelCal, gyroCal);
    ahrs_mahony.begin(10.0F);
};

void loop() {
    imu.Update();
    ahrs.Update();
    cf.Update(imu);

    if (visualise) {
        PrintAhrsVis(ahrs);
    } else {
        Serial.print(micros() / 1000000.0F);
        Serial.print(",");
        PrintImu(imu);
        Serial.print(",");
        PrintAhrs(ahrs);
        Serial.print(",");
        PrintCf(cf);
        Serial.println();
    };

    delay(90);
};