#include "MotorEncoder.h"

// ==========================
// Global Motor Objects
// ==========================

MotorEncoder motorA(
    IN1,
    IN2,
    ENA,
    ENCODER_A_motorA,
    ENCODER_B_motorA);

MotorEncoder motorB(
    IN3,
    IN4,
    ENB,
    ENCODER_A_motorB,
    ENCODER_B_motorB);

// ==========================

MotorEncoder::MotorEncoder(
    int in1,
    int in2,
    int pwmPin,
    int encA,
    int encB)
{
    _in1 = in1;
    _in2 = in2;
    _pwmPin = pwmPin;
    _commandPercent = 0.0f;

    _encA = encA;
    _encB = encB;

    _pulseCount = 0;
    _rawPulseCount = 0;
    _bHighCount = 0;
    _bLowCount = 0;
    _lastIsrUs = 0;
    _lastPulse = 0;
    _lastRawPulse = 0;
    _lastBHigh = 0;
    _lastBLow = 0;

    _rpm = 0;
    _pulsePerSec = 0;
    _rawPulsePerSec = 0;
    _bHighRatio = 0;
    _degPerSec = 0;

    _kp = 0.0f;
    _ki = 0.0f;
    _kd = 0.0f;
    _pidIntegral = 0.0f;
    _pidPrevError = 0.0f;
    _pidOutput = 0.0f;
    _pidLastUs = 0;
}

void MotorEncoder::begin()
{
    pinMode(_in1, OUTPUT);
    pinMode(_in2, OUTPUT);

    pinMode(_encA, INPUT_PULLUP);
    pinMode(_encB, INPUT_PULLUP);

    ledcAttach(
        _pwmPin,
        PWM_FREQ,
        PWM_RESOLUTION);

    _lastIsrUs = micros();

    _lastTime = millis();
    _pidLastUs = micros();
}

void IRAM_ATTR MotorEncoder::encoderISR()
{
    unsigned long nowUs = micros();
    if ((nowUs - _lastIsrUs) < ENCODER_DEBOUNCE_US)
        return;

    bool bHigh = digitalRead(_encB);

    _rawPulseCount++;
    if (bHigh)
        _bHighCount++;
    else
        _bLowCount++;

    // Chi dem canh len A (1x). B chi dung de xac dinh chieu.
    if (bHigh)
        _pulseCount -= ENCODER_SIGN;
    else
        _pulseCount += ENCODER_SIGN;

    _lastIsrUs = nowUs;
}

void MotorEncoder::update()
{
    unsigned long now = millis();

    float dt =
        (now - _lastTime) /
        1000.0f;

    if (dt < 0.05f)
        return;

    long pulseNow;
    long rawPulseNow;
    long bHighNow;
    long bLowNow;
    noInterrupts();
    pulseNow = _pulseCount;
    rawPulseNow = _rawPulseCount;
    bHighNow = _bHighCount;
    bLowNow = _bLowCount;
    interrupts();

    long dp = pulseNow - _lastPulse;
    long rawDp = rawPulseNow - _lastRawPulse;
    long bHighDp = bHighNow - _lastBHigh;
    long bLowDp = bLowNow - _lastBLow;
    long bTotalDp = bHighDp + bLowDp;

    _rawPulsePerSec = (float)rawDp / dt;
    _bHighRatio = (bTotalDp > 0) ? ((float)bHighDp / (float)bTotalDp) : 0.0f;

    // Raw A pulse/s la toc do that. Khi motor dang duoc dieu khien,
    // dau RPM lay theo lenh PWM vi kenh B dang khong on dinh tai canh A.
    if (fabsf(_commandPercent) > 0.1f)
    {
        _pulsePerSec =
            (_commandPercent > 0.0f) ?
            _rawPulsePerSec :
            -_rawPulsePerSec;
    }
    else
    {
        _pulsePerSec = (float)dp / dt;
    }

    // RPM = (pulse/s / xung/vong) * 60
    _rpm =
        (_pulsePerSec /
        PULSE_PER_REV) *
        60.0f;

    _degPerSec =
        (_pulsePerSec /
        PULSE_PER_REV) *
        360.0f;

    if (fabsf(_pulsePerSec) < RPM_ZERO_THRESHOLD)
    {
        _pulsePerSec = 0;
        _rpm = 0;
        _degPerSec = 0;
    }

    _lastPulse = pulseNow;
    _lastRawPulse = rawPulseNow;
    _lastBHigh = bHighNow;
    _lastBLow = bLowNow;
    _lastTime = now;
}

void MotorEncoder::setPWM(float percent)
{
    percent =
        constrain(
            percent,
            -100.0f,
            100.0f);

    _commandPercent = percent;

    int pwm =
        (int)(abs(percent) *
        255.0f /
        100.0f);

    if (percent > 0)
    {
        digitalWrite(_in1, HIGH);
        digitalWrite(_in2, LOW);
    }
    else if (percent < 0)
    {
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, HIGH);
    }
    else
    {
        digitalWrite(_in1, LOW);
        digitalWrite(_in2, LOW);
    }

    ledcWrite(_pwmPin, pwm);
}

void MotorEncoder::setPIDGains(float kp, float ki, float kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
    _pidIntegral = 0.0f;
    _pidPrevError = 0.0f;
    _pidOutput = 0.0f;
    _pidLastUs = micros();
}

float MotorEncoder::controlRPM(float targetRpm)
{
    if (fabsf(targetRpm) <= RPM_TARGET_DEADZONE)
    {
        resetPid();
        return 0.0f;
    }

    unsigned long nowUs = micros();
    float dt = (nowUs - _pidLastUs) / 1000000.0f;
    if (dt <= 0.0005f)
        return _pidOutput;
    _pidLastUs = nowUs;

    float error = targetRpm - _rpm;
    _pidIntegral += error * dt;
    _pidIntegral = constrain(_pidIntegral, -150.0f, 150.0f);

    float derivative = (error - _pidPrevError) / dt;
    _pidPrevError = error;

    _pidOutput =
        (_kp * error) +
        (_ki * _pidIntegral) +
        (_kd * derivative);

    _pidOutput = constrain(_pidOutput, -100.0f, 100.0f);
    setPWM(_pidOutput);
    return _pidOutput;
}

void MotorEncoder::resetPid()
{
    _pidIntegral = 0.0f;
    _pidPrevError = 0.0f;
    _pidOutput = 0.0f;
    _pidLastUs = micros();
    setPWM(0.0f);
}

long MotorEncoder::getPulse()
{
    long count;
    noInterrupts();
    count = _pulseCount;
    interrupts();
    return count;
}

float MotorEncoder::getAngleDeg()
{
    long count;
    noInterrupts();
    count = _pulseCount;
    interrupts();

    return
        ((float)count *
        360.0f) /
        PULSE_PER_REV;
}

float MotorEncoder::getRPM()
{
    return _rpm;
}

float MotorEncoder::getPulsePerSec()
{
    return _pulsePerSec;
}

float MotorEncoder::getRawPulsePerSec()
{
    return _rawPulsePerSec;
}

float MotorEncoder::getBHighRatio()
{
    return _bHighRatio;
}

float MotorEncoder::getDegPerSec()
{
    return _degPerSec;
}

void MotorEncoder::resetEncoder()
{
    noInterrupts();
    _pulseCount = 0;
    _rawPulseCount = 0;
    _bHighCount = 0;
    _bLowCount = 0;
    _lastIsrUs = micros();
    interrupts();

    _lastPulse = 0;
    _lastRawPulse = 0;
    _lastBHigh = 0;
    _lastBLow = 0;
    _rpm = 0;
    _pulsePerSec = 0;
    _rawPulsePerSec = 0;
    _bHighRatio = 0;
    _degPerSec = 0;
    _pidIntegral = 0;
    _pidPrevError = 0;
    _pidOutput = 0;
    _pidLastUs = micros();
}

static float g_joystickSpeedPercent = 40.0f;
static float g_joystickK = JOYSTICK_K_DEFAULT;

void joystickSetSpeedPercent(float percent)
{
    if (percent < 0.0f)
        percent = 0.0f;
    if (percent > 100.0f)
        percent = 100.0f;
    g_joystickSpeedPercent = percent;
}

float joystickGetSpeedPercent()
{
    return g_joystickSpeedPercent;
}

float joystickGetEffectiveV()
{
    return JOYSTICK_V_DEFAULT * g_joystickSpeedPercent / 100.0f;
}

void joystickSetK(float k)
{
    if (k < JOYSTICK_K_MIN)
        k = JOYSTICK_K_MIN;
    if (k > JOYSTICK_K_MAX)
        k = JOYSTICK_K_MAX;
    g_joystickK = k;
}

float joystickGetK()
{
    return g_joystickK;
}

void normalizeJoystickInput(
    float &angleDeg,
    float &magnitude)
{
    while (angleDeg >= 360.0f)
        angleDeg -= 360.0f;
    while (angleDeg < 0.0f)
        angleDeg += 360.0f;

    // 180..360: alpha = 360 - alpha, magnitude doi dau
    if (angleDeg >= 180.0f)
    {
        angleDeg = 360.0f - angleDeg;
        magnitude = -magnitude;
    }
}

void joystickToRpmTargets(
    float angleDeg,
    float magnitude,
    float &rpmA,
    float &rpmB)
{
    rpmA = 0.0f;
    rpmB = 0.0f;

    if (fabsf(magnitude) <= 0.001f)
        return;

    float alphaRad = angleDeg * (PI / 180.0f);
    float tanA = tanf(alphaRad);

    float invKTan = 0.0f;
    if (fabsf(tanA) >= JOYSTICK_TAN_MIN)
    {
        invKTan = (1.0f + fabsf(tanA)) / (joystickGetK() * tanA);
        if (invKTan > 1.0f)
            invKTan = 1.0f;
        else if (invKTan < -1.0f)
            invKTan = -1.0f;
    }

    float vMax = joystickGetEffectiveV();
    rpmA = vMax * (1.0f + invKTan) * magnitude;
    rpmB = vMax * (1.0f - invKTan) * magnitude;
}

// ==========================
// ISR Wrappers
// ==========================

void IRAM_ATTR encoderA_ISR()
{
    motorA.encoderISR();
}

void IRAM_ATTR encoderB_ISR()
{
    motorB.encoderISR();
}
