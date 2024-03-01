//
// Created by Ozan Gerger on 29/02/2024.
//

#include "imu.h"

class LowPassFilter;
class HighPassFilter;

void SetupImu(imu_T& imu, unsigned long baudrate) {
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

void PrintImu(const imu_T& imu) {
    // Display the results (acceleration is converted to m/s^2)
    //Serial.print("\t\tAccel X:");
    Serial.print(imu.filtered_data.accel.acceleration.x);
    Serial.print(",");
    //Serial.print("\tY:");
    Serial.print(imu.filtered_data.accel.acceleration.y);
    Serial.print(",");
    //Serial.print("\tZ:");
    Serial.print(imu.filtered_data.accel.acceleration.z);
    Serial.print(",");

    // Display the results (rotational speed is converted to rad/s)
    //Serial.print("GyX:");
    Serial.print(imu.filtered_data.gyro.gyro.x);
    Serial.print(",");
    //Serial.print(",GyY:");
    Serial.print(imu.filtered_data.gyro.gyro.y);
    Serial.print(",");
   // Serial.print(",GyZ:");
    Serial.print(imu.filtered_data.gyro.gyro.z);
    Serial.print(",");
};

void imu_T::Update() {
    const float range_gyro = 250.0F * DEG_TO_RAD;

    if(sox.accelerationAvailable()){
        sox.readAcceleration(raw_data.accel.acceleration.x, raw_data.accel.acceleration.y, raw_data.accel.acceleration.z);
        raw_data.accel.acceleration.x += calibration.offsAccel[0];
        raw_data.accel.acceleration.y += calibration.offsAccel[1];
        raw_data.accel.acceleration.z += calibration.offsAccel[2];
        raw_data.accel.acceleration.x *= SENSORS_GRAVITY_EARTH;
        raw_data.accel.acceleration.y *= SENSORS_GRAVITY_EARTH;
        raw_data.accel.acceleration.z *= SENSORS_GRAVITY_EARTH;
    }
    if(sox.gyroscopeAvailable())
    {
        sox.readGyroscope(raw_data.gyro.gyro.x, raw_data.gyro.gyro.y, raw_data.gyro.gyro.z);
        raw_data.gyro.gyro.x *= DEG_TO_RAD;
        raw_data.gyro.gyro.y *= DEG_TO_RAD;
        raw_data.gyro.gyro.z *= DEG_TO_RAD;
        raw_data.gyro.gyro.x += calibration.offsGyro[0];
        raw_data.gyro.gyro.y += calibration.offsGyro[1];
        raw_data.gyro.gyro.z += calibration.offsGyro[2];

        raw_data.gyro.gyro.x = std::max(std::min(raw_data.gyro.gyro.x, range_gyro), -range_gyro);
        raw_data.gyro.gyro.y = std::max(std::min(raw_data.gyro.gyro.y, range_gyro), -range_gyro);
        raw_data.gyro.gyro.z = std::max(std::min(raw_data.gyro.gyro.z, range_gyro), -range_gyro);
    }

    this->Filter();
};

void imu_T::Filter() {
    filters.LPaccX.Update(raw_data.accel.acceleration.x);
    filters.LPaccY.Update(raw_data.accel.acceleration.y);
    filters.LPaccZ.Update(raw_data.accel.acceleration.z);

    filters.LPgyX.Update(raw_data.gyro.gyro.x);
    filters.LPgyY.Update(raw_data.gyro.gyro.y);
    filters.LPgyZ.Update(raw_data.gyro.gyro.z);

    filters.HPgyX.Update(filters.LPgyX.GetOutput());
    filters.HPgyY.Update(filters.LPgyY.GetOutput());
    filters.HPgyZ.Update(filters.LPgyZ.GetOutput());

    filtered_data.accel.acceleration.x = filters.LPaccX.GetOutput();
    filtered_data.accel.acceleration.y = filters.LPaccY.GetOutput();
    filtered_data.accel.acceleration.z = filters.LPaccZ.GetOutput();

    filtered_data.gyro.gyro.x = filters.HPgyX.GetOutput();
    filtered_data.gyro.gyro.y = filters.HPgyY.GetOutput();
    filtered_data.gyro.gyro.z = filters.HPgyZ.GetOutput();
}

void imu_T::SetCalibration(const float acc[3], const float gyro[3]) {
    for (int i = 0; i < 3 ; i++){
        calibration.offsAccel[i] = acc[i];
        calibration.offsGyro[i] = gyro[i];
    }
}
