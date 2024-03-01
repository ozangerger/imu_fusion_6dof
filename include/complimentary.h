//
// Created by Ozan Gerger on 01/03/2024.
//

#ifndef IMU_FUSION_6DOF_COMPLIMENTARY_H
#define IMU_FUSION_6DOF_COMPLIMENTARY_H

#include "imu.h"
#include "../LowPassFilter/LowPassFilter.h"

class complFilt_T {
private:
    float Kp{};
    float Ts{};
    struct data {
        float pitch{};
        float roll{};
    } data;

    struct filters {
        explicit filters(float Ts) : Ts(Ts){};
        float Ts;

        LowPassFilter pitch{20.0F * 2.0F * M_PI, Ts};
        LowPassFilter roll{20.0F * 2.0F * M_PI, Ts};
    } filters{Ts};

    float prevPitch{};
    float prevRoll{};

public:
    complFilt_T(float Kp, float w_c, float Ts) : Kp(Kp), Ts(Ts) {
        filters.pitch.Reconfigure(w_c, Ts);
        filters.roll.Reconfigure(w_c, Ts);
    };

    void Update(const imu_T& imu) {
        data.pitch = prevPitch + (imu.filtered_data.gyro.gyro.x + prevRoll *
                                  imu.filtered_data.gyro.gyro.z) * Ts +
                            Kp * (imu.filtered_data.accel.acceleration.x - prevPitch);

        data.roll  = prevRoll + (imu.filtered_data.gyro.gyro.y - prevPitch
                              *  imu.filtered_data.gyro.gyro.z) * Ts +
                           Kp * (imu.filtered_data.accel.acceleration.y - prevRoll);

        filters.pitch.Update(std::max(std::min(data.pitch, 180.0F), -180.0F));
        filters.roll.Update(std::max(std::min(data.roll, 180.0F), -180.0F));

        data.pitch = filters.pitch.GetOutput();
        data.roll = filters.roll.GetOutput();

        prevRoll   = data.roll;
        prevPitch  = data.pitch;
    }

    float GetRollOutput() const { return data.roll; };

    float GetPitchOutput() const { return data.pitch; };
};

void PrintCf(const complFilt_T& cf) {
    Serial.print("\t\tpitch -  complimentary: ");
    Serial.print(cf.GetPitchOutput());
    Serial.print("\troll - complimentary: ");
    Serial.print(cf.GetRollOutput());
    Serial.println();
}

#endif //IMU_FUSION_6DOF_COMPLIMENTARY_H
