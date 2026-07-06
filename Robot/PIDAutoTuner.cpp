#include "PIDAutoTuner.h"

PIDAutoTuner::PIDAutoTuner(
    const float *setpoints,
    int setpointCount,
    unsigned long segmentDurationMs,
    PIDGains initialGains,
    int motorPwmSign)
{
    _setpoints = setpoints;
    _setpointCount = setpointCount;
    _segmentDurationMs = segmentDurationMs;

    _gains = initialGains;
    _targetRpm = (_setpointCount > 0) ? _setpoints[0] : 0.0f;
    _outputPwm = 0.0f;
    _appliedPwm = 0.0f;
    _motorPwmSign = (motorPwmSign >= 0) ? 1 : -1;

    _integral = 0.0f;
    _prevError = 0.0f;
    _lastControlUs = 0;
    _segmentStartMs = 0;
    _setpointIndex = 0;

    _absErrSum = 0.0f;
    _sampleCount = 0;
    _peakAbsRpm = 0.0f;

    _newReport = false;
    _lastReport = {0, 0, 0, _gains, false};
}

void PIDAutoTuner::begin(MotorEncoder &motor)
{
    motor.setPWM(0);
    motor.resetEncoder();
    _integral = 0.0f;
    _prevError = 0.0f;
    _lastControlUs = micros();
    _segmentStartMs = millis();
    _setpointIndex = 0;
    _targetRpm = (_setpointCount > 0) ? _setpoints[0] : 0.0f;
    _outputPwm = 0.0f;
    _appliedPwm = 0.0f;
    resetSegmentStats();
    _newReport = false;
}

void PIDAutoTuner::update(MotorEncoder &motor)
{
    if (_setpointCount <= 0)
        return;

    unsigned long nowUs = micros();
    float dt = (nowUs - _lastControlUs) / 1000000.0f;
    if (dt <= 0.0005f)
        return;
    _lastControlUs = nowUs;

    float rpm = motor.getRPM();
    float err = _targetRpm - rpm;

    _integral += err * dt;
    _integral = clampf(_integral, -300.0f, 300.0f);

    float derivative = (err - _prevError) / dt;
    _prevError = err;

    float pidOut =
        (_gains.kp * err) +
        (_gains.ki * _integral) +
        (_gains.kd * derivative);

    _outputPwm = clampf(pidOut, -100.0f, 100.0f);
    _appliedPwm = _outputPwm * _motorPwmSign;
    motor.setPWM(_appliedPwm);

    float absErr = fabsf(err);
    _absErrSum += absErr;
    _sampleCount++;
    if (fabsf(rpm) > _peakAbsRpm)
        _peakAbsRpm = fabsf(rpm);

    unsigned long nowMs = millis();
    if ((nowMs - _segmentStartMs) >= _segmentDurationMs)
    {
        float meanAbsError = (_sampleCount > 0) ? (_absErrSum / _sampleCount) : 0.0f;
        float targetAbs = fabsf(_targetRpm);
        float overshootRatio = 0.0f;
        if (targetAbs > 1.0f && _peakAbsRpm > targetAbs)
            overshootRatio = (_peakAbsRpm - targetAbs) / targetAbs;

        _lastReport.targetRpm = _targetRpm;
        _lastReport.meanAbsError = meanAbsError;
        _lastReport.overshootRatio = overshootRatio;
        _lastReport.gains = _gains;
        _lastReport.valid = true;
        _newReport = true;

        applyAdaptiveRule();

        _setpointIndex = (_setpointIndex + 1) % _setpointCount;
        _targetRpm = _setpoints[_setpointIndex];
        _integral = 0.0f;
        _prevError = _targetRpm - rpm;
        _segmentStartMs = nowMs;
        resetSegmentStats();
    }
}

float PIDAutoTuner::getTargetRpm() const
{
    return _targetRpm;
}

float PIDAutoTuner::getOutputPwm() const
{
    return _outputPwm;
}

float PIDAutoTuner::getAppliedPwm() const
{
    return _appliedPwm;
}

PIDGains PIDAutoTuner::getGains() const
{
    return _gains;
}

bool PIDAutoTuner::hasNewReport() const
{
    return _newReport;
}

TuneReport PIDAutoTuner::consumeReport()
{
    _newReport = false;
    return _lastReport;
}

void PIDAutoTuner::resetSegmentStats()
{
    _absErrSum = 0.0f;
    _sampleCount = 0;
    _peakAbsRpm = 0.0f;
}

void PIDAutoTuner::applyAdaptiveRule()
{
    float meanAbsError = (_sampleCount > 0) ? (_absErrSum / _sampleCount) : 0.0f;
    float targetAbs = fabsf(_targetRpm);
    float overshootRatio = 0.0f;
    if (targetAbs > 1.0f && _peakAbsRpm > targetAbs)
        overshootRatio = (_peakAbsRpm - targetAbs) / targetAbs;

    if (meanAbsError > 12.0f)
    {
        _gains.kp *= 1.08f;
        _gains.ki *= 1.06f;
    }
    else if (meanAbsError < 4.0f && overshootRatio < 0.05f)
    {
        _gains.ki *= 0.98f;
    }

    if (overshootRatio > 0.25f)
    {
        _gains.kp *= 0.90f;
        _gains.kd *= 1.15f;
    }
    else if (overshootRatio > 0.10f)
    {
        _gains.kp *= 0.95f;
        _gains.kd *= 1.08f;
    }
    else if (overshootRatio < 0.03f && meanAbsError > 6.0f)
    {
        _gains.kp *= 1.03f;
    }

    _gains.kp = clampf(_gains.kp, 0.001f, 10.0f);
    _gains.ki = clampf(_gains.ki, 0.0f, 20.0f);
    _gains.kd = clampf(_gains.kd, 0.0f, 2.0f);
    _gains.valid = true;
}

float PIDAutoTuner::clampf(float x, float lo, float hi) const
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}
