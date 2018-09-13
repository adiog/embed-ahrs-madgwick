// This file is a part of embed-sensor-fusion project.
// Copyright 2018 Aleksander Gajewski <adiog@brainfuck.pl>.

#pragma once

#include <SensorFusion.h>

#define sampleFreq 512.0f  // sample frequency in Hz
#define betaDef 0.1f       // 2 * proportional gain


namespace sensorFusion {
struct Madgwick : public SensorFusion
{
    void initialize(const SensorData &sensorData) override
    {
    }

    FusionData apply(const SensorData &sensorData) override
    {
        update(sensorData);
        computeAngles();
        return FusionData{{yaw, pitch, roll}};
    }

    Float beta = betaDef;                              // 2 * proportional gain (Kp)
    Float q0 = 1.0;
    Float q1 = 0.0;
    Float q2 = 0.0;
    Float q3 = 0.0f;  // quaternion of sensor frame relative to auxiliary frame

    Float yaw;
    Float pitch;
    Float roll;

    void update(const sensorFusion::SensorData &sensorData);
    void updateImu(const sensorFusion::SensorData &sensorData);
    void computeAngles();
};
}
