//
// Created by Ozan Gerger on 29/02/2024.
//

#ifndef QUADCOPTER_SENSOR_FUSION_H
#define QUADCOPTER_SENSOR_FUSION_H

#include "SensorFusion.h"

class imu_T;

class sensorFusion_T {
public:
    sensorFusion_T(const imu_T& imu, SF& sf) : imu(imu), sf(sf) {};
    const imu_T& imu;
    SF& sf;
    void update();

    float roll{};
    float pitch{};
    float yaw{};
};

void print_sf(const sensorFusion_T& sf);

#endif //QUADCOPTER_SENSOR_FUSION_H
