#ifndef MOTOR_ENCODER_H
#define MOTOR_ENCODER_H

#include <Arduino.h>

/*
==================================================
HARDWARE CONFIG
==================================================
Chỉ sửa phần này khi đổi phần cứng
==================================================
*/

// =======================
// Motor A
// =======================

#define IN1 26
#define IN2 27
#define ENA 25

#define ENCODER_A_motorA 2
#define ENCODER_B_motorA 4

// =======================
// Motor B
// =======================

#define IN3 14
#define IN4 12
#define ENB 13

#define ENCODER_A_motorB 16
#define ENCODER_B_motorB 17

// =======================
// Encoder Config
// =======================

// Encoder pulse/vòng trục motor
#define ENCODER_PPR 11.0f

// Tỉ số truyền
#define GEAR_RATIO 46.8f

// Đếm 1 cạnh RISING trên kênh A
#define ENCODER_MULTIPLIER 1.0f

// Pulse/vòng trục ra = 11 * 46.8 = 514.8 (float, khong lam tron)
#define PULSE_PER_REV \
(ENCODER_PPR * GEAR_RATIO * ENCODER_MULTIPLIER)

// 1 = bình thường, -1 = đảo chiều đếm xung nếu dây encoder lắp ngược
#define ENCODER_SIGN 1

// Bo qua canh xung qua gan nhau (us) - chong nhieu/PWM
#define ENCODER_DEBOUNCE_US 80

// |pulse/s| nho hon nguong nay coi nhu dung (chi de hien thi RPM)
#define RPM_ZERO_THRESHOLD 2.0f

// =======================
// PWM Config
// =======================

#define PWM_FREQ       5000
#define PWM_RESOLUTION 8

// =======================
// Fixed PID Gains
// =======================

#define MOTOR_A_KP 1.2991f
#define MOTOR_A_KI 2.2918f
#define MOTOR_A_KD 0.1627f

#define MOTOR_B_KP 1.3298f
#define MOTOR_B_KI 2.1621f
#define MOTOR_B_KD 0.1329f

// =======================
// Joystick -> RPM
// =======================

#define JOYSTICK_V_DEFAULT 100.0f
#define JOYSTICK_K_DEFAULT 10.0f
#define JOYSTICK_K_MIN 1.0f
#define JOYSTICK_K_MAX 20.0f

void joystickSetSpeedPercent(float percent);
float joystickGetSpeedPercent();
float joystickGetEffectiveV();

void joystickSetK(float k);
float joystickGetK();

// Tranh chia cho tan(alpha) ~ 0
#define JOYSTICK_TAN_MIN 0.05f

// |target rpm| nho hon nguong nay: pwm=0 va reset PID
#define RPM_TARGET_DEADZONE 5.0f

void normalizeJoystickInput(
    float &angleDeg,
    float &magnitude);

void joystickToRpmTargets(
    float angleDeg,
    float magnitude,
    float &rpmA,
    float &rpmB);

class MotorEncoder
{
public:

    MotorEncoder(
        int in1,
        int in2,
        int pwmPin,
        int encA,
        int encB);

    void begin();

    void encoderISR();

    void update();

    void setPWM(float percent);

    void setPIDGains(float kp, float ki, float kd);

    float controlRPM(float targetRpm);

    long getPulse();

    float getAngleDeg();

    float getRPM();

    float getPulsePerSec();

    float getRawPulsePerSec();

    float getBHighRatio();

    float getDegPerSec();

    void resetEncoder();

    void resetPid();

private:

    int _in1;
    int _in2;
    int _pwmPin;
    float _commandPercent;

    int _encA;
    int _encB;

    volatile long _pulseCount;

    volatile long _rawPulseCount;

    volatile long _bHighCount;

    volatile long _bLowCount;

    volatile unsigned long _lastIsrUs;

    long _lastPulse;

    long _lastRawPulse;

    long _lastBHigh;

    long _lastBLow;

    unsigned long _lastTime;

    float _rpm;

    float _pulsePerSec;

    float _rawPulsePerSec;

    float _bHighRatio;

    float _degPerSec;

    float _kp;
    float _ki;
    float _kd;
    float _pidIntegral;
    float _pidPrevError;
    float _pidOutput;
    unsigned long _pidLastUs;
};

extern MotorEncoder motorA;
extern MotorEncoder motorB;

void IRAM_ATTR encoderA_ISR();
void IRAM_ATTR encoderB_ISR();

#endif
