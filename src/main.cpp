#include <Arduino.h>
#include "imu.h"
#include "sensor_fusion.h"

SF fusion{0.5F, 0.1F};
Adafruit_LSM6DSOX sox;
imu_T imu(sox, 0.1F, 20.0F * 2.0F * M_PI, 5.0F * 2.0F * M_PI);
sensorFusion_T sf(imu, fusion);
float accelCal[3] = {0.0F,0.0F,0.0F};
float gyroCal[3] = {-10.2F,0.2F,-2.2F};

void setup() {
    unsigned long baudrate = 115200;
    setup_imu(imu, baudrate);
    imu.set_calibration(accelCal, gyroCal);
}

void loop() {
    imu.update();

    sf.delta_t = sf.sf.deltatUpdate();
    //Serial.println(sf.delta_t);

    sf.update();
    print_imu(imu);
    print_sf(sf);

    delay(90);
};