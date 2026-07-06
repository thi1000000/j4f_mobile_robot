#include "Odometry.h"
#include "MotorEncoder.h"

static float g_x = 0.0f;
static float g_y = 0.0f;
static float g_theta = 0.0f;

static unsigned long g_lastUpdateUs = 0;

static const float METERS_PER_REV =
    PI * ODOM_WHEEL_DIAMETER_M;

static float rpmToMps(float rpm)
{
    return rpm * METERS_PER_REV / 60.0f;
}

void odometryBegin()
{
    g_x = 0.0f;
    g_y = 0.0f;
    g_theta = 0.0f;
    g_lastUpdateUs = micros();
}

void odometryReset()
{
    odometryBegin();
}

void odometryUpdate()
{
    unsigned long nowUs = micros();
    float dt = (nowUs - g_lastUpdateUs) / 1000000.0f;
    if (dt < 0.02f)
        return;
    g_lastUpdateUs = nowUs;

    float vLeft = rpmToMps(motorA.getRPM() * ODOM_MOTOR_A_SIGN);
    float vRight = rpmToMps(motorB.getRPM() * ODOM_MOTOR_B_SIGN);

    float v = (vLeft + vRight) * 0.5f;
    float omega = (vRight - vLeft) / ODOM_WHEEL_BASE_M;

    if (fabsf(v) < 1e-6f && fabsf(omega) < 1e-6f)
        return;

    // x(k+1) = x(k) + v*Ts*cos(phi),  y(k+1) = y(k) + v*Ts*sin(phi),  phi(k+1) = phi(k) + omega*Ts
    g_x += v * dt * cosf(g_theta);
    g_y += v * dt * sinf(g_theta);
    g_theta += omega * dt;

    while (g_theta > PI)
        g_theta -= 2.0f * PI;
    while (g_theta < -PI)
        g_theta += 2.0f * PI;
}

float odometryGetX()
{
    return g_x;
}

float odometryGetY()
{
    return g_y;
}

float odometryGetThetaRad()
{
    return g_theta;
}

float odometryGetThetaDeg()
{
    return g_theta * (180.0f / PI);
}
