#include "odom_calib/robot_params.hpp"
#include "odom_calib/math.hpp"
#include <cmath>

long correct_overflow(long diff, const RobotParams& p) {
    if (diff > p.ENCODER_THRESHOLD) {
        return diff - p.ENCODER_MAX;
    }
    if (diff < -p.ENCODER_THRESHOLD) {
        return diff + p.ENCODER_MAX;
    }

    return diff;
}

double ticks_to_meters(long diff, const RobotParams& p) {
    return (static_cast<double>(diff)/ p.N) * 2.0 * M_PI * p.R;
}

double calculate_distance(double x1, double y1, double x2, double y2) {
    return sqrt(pow(x2-x1, 2) + pow(y2-y1, 2));
}

// Новая функция: Кватернион -> Угол Yaw
double quaternion_to_yaw(double x, double y, double z, double w) {
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    return std::atan2(siny_cosp, cosy_cosp);
}

// Новая функция: Правильная разница углов (с учетом перехода через PI)
double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}