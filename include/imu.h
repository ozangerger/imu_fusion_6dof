//
// Created by Ozan Gerger on 29/02/2024.
//

#ifndef IMU_FUSION_6DOF_IMU_H
#define IMU_FUSION_6DOF_IMU_H

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LSM6DS.h>
#include "../lib/LowPassFilter/LowPassFilter.h"
#include "../lib/HighPassFilter/HighPassFilter.h"
#include <cmath>

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

class imu_T {
private:
    float Ts = 0.01F;
    struct calibration {
        float offsAccel[3];
        float offsGyro[3];
    } calibration{};

    struct settings_T {
        lsm6ds_gyro_range_t gyro_range = LSM6DS_GYRO_RANGE_2000_DPS;
        lsm6ds_accel_range_t accel_range = LSM6DS_ACCEL_RANGE_4_G;
        lsm6ds_data_rate_t accel_data_rate = LSM6DS_RATE_416_HZ;
        lsm6ds_data_rate_t gyro_data_rate = LSM6DS_RATE_416_HZ;
        lsm6ds_hp_filter_t hp_filter = LSM6DS_HPF_ODR_DIV_100;
    } settings;

public:
    explicit imu_T(Adafruit_LSM6DSOX &imu, float Ts, float wc_accel_lp, float wc_gy_lp, float wc_gy_hp) :
            Ts(Ts), sox(imu) {
        filters.LPaccX.Reconfigure(wc_accel_lp, Ts);
        filters.LPaccY.Reconfigure(wc_accel_lp, Ts);
        filters.LPaccZ.Reconfigure(wc_accel_lp, Ts);
        filters.LPgyX.Reconfigure(wc_gy_lp, Ts);
        filters.LPgyY.Reconfigure(wc_gy_lp, Ts);
        filters.LPgyZ.Reconfigure(wc_gy_lp, Ts);
        filters.HPgyX.Reconfigure(wc_gy_hp, Ts);
        filters.HPgyY.Reconfigure(wc_gy_hp, Ts);
        filters.HPgyZ.Reconfigure(wc_gy_hp, Ts);
    };

    struct filters {
        explicit filters(float Ts) : Ts(Ts) {};
        float Ts;

        LowPassFilter LPaccX{20.0F * 2.0F * M_PI, Ts};
        LowPassFilter LPaccY{20.0F * 2.0F * M_PI, Ts};
        LowPassFilter LPaccZ{20.0F * 2.0F * M_PI, Ts};
        LowPassFilter LPgyX{20.0F * 2.0F * M_PI, Ts};
        LowPassFilter LPgyY{20.0F * 2.0F * M_PI, Ts};
        LowPassFilter LPgyZ{20.0F * 2.0F * M_PI, Ts};
        HighPassFilter HPgyX{0.05F * 2.0F * M_PI, Ts};
        HighPassFilter HPgyY{0.05F * 2.0F * M_PI, Ts};
        HighPassFilter HPgyZ{0.05F * 2.0F * M_PI, Ts};
    } filters{Ts};

    bool serial = false;
    Adafruit_LSM6DSOX &sox;

    void Update();

    settings_T GetSettings() { return this->settings; };

    void Filter();

    void SetCalibration(const float acc[3], const float gyro[3]);

    struct data {
        sensors_event_t accel{};
        sensors_event_t gyro{};
    } raw_data, filtered_data;
};

void SetupImu(imu_T &imu, unsigned long baudrate);

void PrintImu(const imu_T &);

#endif //IMU_FUSION_6DOF_IMU_H
