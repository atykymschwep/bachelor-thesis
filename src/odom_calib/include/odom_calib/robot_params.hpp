#pragma once

struct RobotParams
{
    double R = 0.0363; // 0.035
    double B = 0.2275; // 0.230
    double N  = 2578.33;
    const int ENCODER_MAX = 65536;
    const int ENCODER_THRESHOLD = 32768;
};
