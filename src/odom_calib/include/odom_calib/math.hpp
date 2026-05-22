#pragma once
#include "robot_params.hpp"

long correct_overflow(long diff, const RobotParams& p);

double ticks_to_meters(long diff, const RobotParams& p);

double calculate_distance(double x1, double y1, double x2, double y2);

double quaternion_to_yaw(double x, double y, double z, double w);

double normalize_angle(double angle);