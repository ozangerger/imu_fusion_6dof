#include <Arduino.h>
#include "imu.h"
#include "sensor_fusion.h"

SF fusion;
Adafruit_LSM6DSOX sox;
imu_T imu(sox, 0.01F, 50.0F);
sensorFusion_T sf(imu, fusion);

void setup() {
    unsigned long baudrate = 115200;
    setup_imu(imu, baudrate);
}

void loop() {

    imu.update();

    sf.update();

    print_imu(imu);

    print_sf(sf);

    delay(100);
};