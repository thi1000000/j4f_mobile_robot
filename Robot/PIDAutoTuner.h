#ifndef PID_AUTO_TUNER_H
#define PID_AUTO_TUNER_H

#include <Arduino.h>
#include "MotorEncoder.h"

struct PIDGains
{
    float kp;
    float ki;
    float kd;
    float ku;
    float pu;
    bool valid;
};

struct TuneReport
{
    float targetRpm;
    float meanAbsError;
    float overshootRatio;
    PIDGains gains;
    bool valid;
};

class PIDAutoTuner
{
public:
    PIDAutoTuner(
        const float *setpoints,
        int setpointCount,
        unsigned long segmentDurationMs,
        PIDGains initialGains,
        int motorPwmSign = 1);

    void begin(MotorEncoder &motor);

    void update(MotorEncoder &motor);

    float getTargetRpm() const;

    float getOutputPwm() const;
    float getAppliedPwm() const;

    PIDGains getGains() const;

    bool hasNewReport() const;

    TuneReport consumeReport();

private:
    void resetSegmentStats();

    void applyAdaptiveRule();

    float clampf(float x, float lo, float hi) const;

    const float *_setpoints;
    int _setpointCount;
    unsigned long _segmentDurationMs;

    PIDGains _gains;
    float _targetRpm;
    float _outputPwm;
    float _appliedPwm;
    int _motorPwmSign;

    float _integral;
    float _prevError;
    unsigned long _lastControlUs;
    unsigned long _segmentStartMs;
    int _setpointIndex;

    float _absErrSum;
    int _sampleCount;
    float _peakAbsRpm;

    bool _newReport;
    TuneReport _lastReport;
};

#endif
