//
// Created by Ozan Gerger on 29/02/2024.
//
#include "sensor_fusion.h"
#include "imu.h"

void sensorFusion_T::update(float delta_t) {
    //choose only one of these two:
    sf.MahonyUpdate(imu.filtered_data.gyro.gyro.x, imu.filtered_data.gyro.gyro.y,
                    imu.filtered_data.gyro.gyro.z,imu.filtered_data.accel.acceleration.x,
                    imu.filtered_data.accel.acceleration.y, imu.filtered_data.accel.acceleration.z, delta_t);  //mahony is suggested if there isn't the mag and the mcu is slow
    //fusion.MadgwickUpdate(gyros[0], gyros[1], gyros[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], deltat);  //else use the magwick, it is slower but more accurate
    pitch = sf.getPitch();
    roll = sf.getRoll();    //you could also use getRollRadians() ecc
    yaw = sf.getYaw();
};

void print_sf(const sensorFusion_T& sf) {
    Serial.print("\t\tPitch: ");
    Serial.print(sf.pitch);
    Serial.print(" \tRoll: ");
    Serial.print(sf.roll);
    Serial.print(" \tYaw: ");
    Serial.print(sf.yaw);
    Serial.println(" deg ");
};