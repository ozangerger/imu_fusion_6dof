//
// Created by Ozan Gerger on 29/02/2024.
//

#include "imu.h"

class LowPassFilter;

void setup_imu(imu_T& imu, unsigned long baudrate) {
    Serial.begin(baudrate);
    while (!Serial)
        delay(10);

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
    imu.sox.setGyroDataRate(imu.GetSettings().data_rate);
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

void imu_T::update(void) {
    sox.readAcceleration(raw_data.accel.acceleration.x, raw_data.accel.acceleration.y, raw_data.accel.acceleration.z);
    sox.readGyroscope(raw_data.gyro.gyro.x, raw_data.gyro.gyro.y, raw_data.gyro.gyro.z);
    this->filter();
};

void imu_T::filter(void) {
    filters.accX.Update(g_to_mps(raw_data.accel.acceleration.x));
    filters.accY.Update(g_to_mps(raw_data.accel.acceleration.y));
    filters.accZ.Update(g_to_mps(raw_data.accel.acceleration.z));

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

