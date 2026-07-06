#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>

// Do thuc te tren xe (m) — chinh sau khi test
#define ODOM_WHEEL_DIAMETER_M 0.065f
#define ODOM_WHEEL_BASE_M       0.20f

// motorA = trai, motorB = phai. Doi dau neu chay nguoc.
#define ODOM_MOTOR_A_SIGN 1
#define ODOM_MOTOR_B_SIGN 1

void odometryBegin();

void odometryUpdate();

void odometryReset();

float odometryGetX();

float odometryGetY();

float odometryGetThetaRad();

float odometryGetThetaDeg();

#endif
