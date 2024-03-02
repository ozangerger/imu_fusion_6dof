//
// Created by Ozan Gerger on 29/02/2024.
//
#include "ahrs.h"
#include "imu.h"

void ahrs_T::Update() {
    //choose only one of these two:
    // gyro inputs are rad/s, accelerometer inputs are m/s2

    ahrs.updateIMU(imu.filtered_data.gyro.gyro.x,
                 imu.filtered_data.gyro.gyro.y,
                 imu.filtered_data.gyro.gyro.z,
                 imu.filtered_data.accel.acceleration.x,
                 imu.filtered_data.accel.acceleration.y,
                 imu.filtered_data.accel.acceleration.z);
    //mahony is suggested if there isn't the mag and the mcu is slow
    //sf.update(gyros[0], gyros[1], gyros[2], acc[0], acc[1], acc[2], mag[0], mag[1], mag[2], deltat);  //else use the magwick, it is slower but more accurate
    pitch = ahrs.getPitch();
    roll = ahrs.getRoll();    //you could also use getRollRadians() ecc
    yaw = ahrs.getYaw();
};

void PrintAhrs(const ahrs_T& ahrs) {
    //Serial.print("Pitch:");
    Serial.print(ahrs.pitch);
    Serial.print(",");
    Serial.print(ahrs.roll);
    Serial.print(",");
    Serial.print(ahrs.yaw);
};

void PrintAhrsVis(const ahrs_T& ahrs) {
    Serial.print(ahrs.pitch);
    Serial.print(",");
    Serial.print(ahrs.roll * -1.0);
    Serial.print(",");
    // yaw is way off, setting to 0.0
    Serial.println(0.0);
};