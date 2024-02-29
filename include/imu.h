//
// Created by Ozan Gerger on 29/02/2024.
//

#ifndef QUADCOPTER_IMU_H
#define QUADCOPTER_IMU_H

#include <Adafruit_LSM6DSOX.h>
#include <Adafruit_LSM6DS.h>
#include "../lib/LowPassFilter/LowPassFilter.h"

// For SPI mode, we need a CS pin
#define LSM_CS 10
// For software-SPI mode we need SCK/MOSI/MISO pins
#define LSM_SCK 13
#define LSM_MISO 12
#define LSM_MOSI 11

class imu_T {
private:
    float Ts = 0.01F;

    struct settings_T {
        lsm6ds_gyro_range_t   gyro_range  = LSM6DS_GYRO_RANGE_2000_DPS;
        lsm6ds_accel_range_t  accel_range = LSM6DS_ACCEL_RANGE_8_G;
        lsm6ds_data_rate_t    data_rate   = LSM6DS_RATE_833_HZ;
        lsm6ds_hp_filter_t    hp_filter   = LSM6DS_HPF_ODR_DIV_400;
    } settings;

public:
    explicit imu_T(Adafruit_LSM6DSOX& imu, float Ts, float w_c) :  Ts(Ts), sox(imu)  {
        filters.accX.Reconfigure(w_c, Ts);
        filters.accY.Reconfigure(w_c, Ts);
        filters.accZ.Reconfigure(w_c, Ts);
        filters.gyX.Reconfigure(w_c, Ts);
        filters.gyY.Reconfigure(w_c, Ts);
        filters.gyZ.Reconfigure(w_c, Ts);
    };

    struct filters {
        explicit filters(float Ts) : Ts(Ts){};
        float Ts;

        LowPassFilter accX{20.0F, Ts};
        LowPassFilter accY{20.0F, Ts};
        LowPassFilter accZ{20.0F, Ts};
        LowPassFilter  gyX{20.0F, Ts};
        LowPassFilter  gyY{20.0F, Ts};
        LowPassFilter  gyZ{20.0F, Ts};
    } filters{Ts};

    bool serial = false;
    Adafruit_LSM6DSOX& sox;

    void update();
    settings_T GetSettings() {return this->settings;};

    void filter();

    struct data {
        sensors_event_t accel{};
        sensors_event_t gyro{};
    } raw_data, filtered_data;
};

void setup_imu(imu_T& imu, unsigned long baudrate);

void print_imu(const imu_T&);

#endif //QUADCOPTER_IMU_H
