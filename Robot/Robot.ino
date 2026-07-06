#include "MotorEncoder.h"
#include "PiUart.h"
#include "Odometry.h"

void setup()
{
    Serial.begin(115200);
    delay(1000);

    motorA.begin();
    motorB.begin();

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_A_motorA),
        encoderA_ISR,
        RISING);

    attachInterrupt(
        digitalPinToInterrupt(ENCODER_A_motorB),
        encoderB_ISR,
        RISING);

    motorA.setPIDGains(MOTOR_A_KP, MOTOR_A_KI, MOTOR_A_KD);
    motorB.setPIDGains(MOTOR_B_KP, MOTOR_B_KI, MOTOR_B_KD);

    motorA.setPWM(0);
    motorB.setPWM(0);

    odometryBegin();

    if (!piUartBegin())
    {
        Serial.println("ERROR: Cannot start Pi UART");
        while (true)
            delay(1000);
    }

    Serial.println();
    Serial.println("=== Robot: Pi UART + PID + Odometry ===");
    Serial.println("UART: RX=GPIO18 TX=GPIO19 @ 115200");
}

void loop()
{
    piUartUpdate();

    motorA.update();
    motorB.update();
    odometryUpdate();

    float rawMag = piUartGetMagnitude();
    float rawAngle = piUartGetAngleDeg();
    float angleDeg = rawAngle;
    float magnitude = rawMag;

    normalizeJoystickInput(angleDeg, magnitude);

    float targetA = 0.0f;
    float targetB = 0.0f;
    joystickToRpmTargets(angleDeg, magnitude, targetA, targetB);

    if (piUartIsStopped())
    {
        motorA.resetPid();
        motorB.resetPid();
    }
    else if (fabsf(rawMag) > 0.01f)
    {
        motorA.controlRPM(targetA);
        motorB.controlRPM(targetB);
    }
    else
    {
        motorA.resetPid();
        motorB.resetPid();
    }

    static unsigned long lastStatus = 0;
    if (millis() - lastStatus > 500)
    {
        lastStatus = millis();
        Serial.print("Odom x=");
        Serial.print(odometryGetX(), 3);
        Serial.print(" y=");
        Serial.print(odometryGetY(), 3);
        Serial.print(" th=");
        Serial.print(odometryGetThetaDeg(), 1);
        Serial.print(" | A RPM=");
        Serial.print(motorA.getRPM(), 1);
        Serial.print(" B RPM=");
        Serial.println(motorB.getRPM(), 1);
    }
}
