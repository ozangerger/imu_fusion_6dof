//
// Created by Ozan Gerger on 29/02/2024.
//

#ifndef IMU_FUSION_6DOF_AHRS_H
#define IMU_FUSION_6DOF_AHRS_H

#include "MahonyAHRS.h"

class imu_T;

class ahrs_T {
public:
    ahrs_T(const imu_T& imu, Mahony& ahrs) : imu(imu), ahrs(ahrs) {};
    const imu_T& imu;
    Mahony& ahrs;
    void Update();

    float roll{};
    float pitch{};
    float yaw{};
    float delta_t{};
};

void PrintAhrs(const ahrs_T& sf);

#endif //IMU_FUSION_6DOF_AHRS_H
