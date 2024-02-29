//
// Created by Ozan Gerger on 29/02/2024.
//

#include "imu.h"

class LowPassFilter;

void setup_imu(imu_T& imu, unsigned long baudrate) {
    Serial.begin(baudrate);
    /*while (!Serial)
        delay(10);*/

    if (!imu.sox.begin_I2C()) {
        // if (!sox.begin_SPI(LSM_CS)) {
        // if (!sox.begin_SPI(LSM_CS, LSM_SCK, LSM_MISO, LSM_MOSI)) {
        // Serial.println("Failed to find LSM6DSOX chip");
        while (!imu.sox.begin_I2C()) {
            delay(10);
        }
    }

    imu.serial = true;
    Serial.println("LSM6DSOX Found!");

    imu.sox.setAccelRange(imu.GetSettings().accel_range);
    imu.sox.setGyroRange(imu.GetSettings().gyro_range);
    imu.sox.setAccelDataRate(imu.GetSettings().accel_data_rate);
    imu.sox.setGyroDataRate(imu.GetSettings().gyro_data_rate);
    imu.sox.highPassFilter(false, imu.GetSettings().hp_filter);
};

void print_imu(const imu_T& imu) {
    // Display the results (acceleration is measured in m/s^2)
    Serial.print("\t\tAccel X: ");
    Serial.print(imu.filtered_data.accel.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(imu.filtered_data.accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(imu.filtered_data.accel.acceleration.z);
    Serial.println(" m/s^2 ");

    // Display the results (rotation is measured in rad/s)
    Serial.print("\t\tGyro X: ");
    Serial.print(imu.filtered_data.gyro.gyro.x);
    Serial.print(" \tY: ");
    Serial.print(imu.filtered_data.gyro.gyro.y);
    Serial.print(" \tZ: ");
    Serial.print(imu.filtered_data.gyro.gyro.z);
    Serial.println(" radians/s ");
    Serial.println();
};

void imu_T::update() {
    if(sox.accelerationAvailable()){
        sox.readAcceleration(raw_data.accel.acceleration.x, raw_data.accel.acceleration.y, raw_data.accel.acceleration.z);
        raw_data.accel.acceleration.x += calibration.offsAccel[0];
        raw_data.accel.acceleration.y += calibration.offsAccel[1];
        raw_data.accel.acceleration.z += calibration.offsAccel[2];
    }
    if(sox.gyroscopeAvailable())
    {
        sox.readGyroscope(raw_data.gyro.gyro.x, raw_data.gyro.gyro.y, raw_data.gyro.gyro.z);
        raw_data.gyro.gyro.x += calibration.offsGyro[0];
        raw_data.gyro.gyro.y += calibration.offsGyro[1];
        raw_data.gyro.gyro.z += calibration.offsGyro[2];
    }

    this->filter();
};

void imu_T::filter() {
    filters.accX.Update(raw_data.accel.acceleration.x);
    filters.accY.Update(raw_data.accel.acceleration.y);
    filters.accZ.Update(raw_data.accel.acceleration.z);

    filters.gyX.Update(raw_data.gyro.gyro.x);
    filters.gyY.Update(raw_data.gyro.gyro.y);
    filters.gyZ.Update(raw_data.gyro.gyro.z);

    filtered_data.accel.acceleration.x = filters.accX.GetOutput();
    filtered_data.accel.acceleration.y = filters.accY.GetOutput();
    filtered_data.accel.acceleration.z = filters.accZ.GetOutput();

    filtered_data.gyro.gyro.x = filters.gyX.GetOutput();
    filtered_data.gyro.gyro.y = filters.gyY.GetOutput();
    filtered_data.gyro.gyro.z = filters.gyZ.GetOutput();
}

void imu_T::set_calibration(const float acc[3], const float gyro[3]) {
    for (int i = 0; i < 3 ; i++){
        calibration.offsAccel[i] = acc[i];
        calibration.offsGyro[i] = gyro[i];
    }
}
