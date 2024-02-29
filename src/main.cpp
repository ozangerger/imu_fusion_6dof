#include <Arduino.h>
#include "imu.h"
#include "sensor_fusion.h"

SF fusion;
Adafruit_LSM6DSOX sox;
imu_T imu(sox, 0.01F, 20.0F);
sensorFusion_T sf(imu, fusion);

void setup() {
    unsigned long baudrate = 115200;
    setup_imu(imu, baudrate);
}

void loop() {

    imu.update();

    const float deltat = sf.sf.deltatUpdate(); //this have to be done before calling the fusion update

    sf.update(deltat);

    //print_imu(imu);

    print_sf(sf);

    delay(100);
};