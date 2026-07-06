#include "PiUart.h"
#include "Odometry.h"
#include "MotorEncoder.h"

static float g_angleDeg = 0.0f;
static float g_magnitude = 0.0f;
static unsigned long g_lastPacketMs = 0;
static bool g_motorStopped = false;

static char g_rxBuf[96];
static uint8_t g_rxLen = 0;

static unsigned long g_lastOdomSendMs = 0;

static void resetRxBuf()
{
    g_rxLen = 0;
    g_rxBuf[0] = '\0';
}

static void handleLine(const char *line)
{
    if (strncmp(line, "JOY,", 4) == 0)
    {
        float angle = 0.0f;
        float mag = 0.0f;
        if (sscanf(line + 4, "%f,%f", &angle, &mag) == 2)
        {
            g_angleDeg = angle;
            g_magnitude = mag;
            g_motorStopped = false;
            g_lastPacketMs = millis();
        }
        return;
    }

    if (strcmp(line, "STOP") == 0)
    {
        g_motorStopped = true;
        g_angleDeg = 0.0f;
        g_magnitude = 0.0f;
        g_lastPacketMs = millis();
        return;
    }

    if (strncmp(line, "SPD,", 4) == 0)
    {
        joystickSetSpeedPercent(atof(line + 4));
        return;
    }

    if (strncmp(line, "K,", 2) == 0)
    {
        joystickSetK(atof(line + 2));
        return;
    }

    if (strcmp(line, "ODOMRST") == 0)
    {
        odometryReset();
    }
}

static void readIncoming()
{
    while (Serial2.available() > 0)
    {
        char c = (char)Serial2.read();
        if (c == '\r')
            continue;

        if (c == '\n')
        {
            g_rxBuf[g_rxLen] = '\0';
            if (g_rxLen > 0)
                handleLine(g_rxBuf);
            resetRxBuf();
            continue;
        }

        if (g_rxLen < sizeof(g_rxBuf) - 1)
            g_rxBuf[g_rxLen++] = c;
        else
            resetRxBuf();
    }
}

static void sendOdom()
{
    unsigned long now = millis();
    if ((now - g_lastOdomSendMs) < 100)
        return;
    g_lastOdomSendMs = now;

    Serial2.printf(
        "ODOM,%.4f,%.4f,%.2f,%.1f,%.1f,%ld,%ld\n",
        odometryGetX(),
        odometryGetY(),
        odometryGetThetaDeg(),
        motorA.getRPM(),
        motorB.getRPM(),
        motorA.getPulse(),
        motorB.getPulse());
}

bool piUartBegin()
{
    Serial2.begin(PI_UART_BAUD, SERIAL_8N1, PI_UART_RX, PI_UART_TX);
    resetRxBuf();
    g_lastOdomSendMs = 0;
    return true;
}

void piUartUpdate()
{
    readIncoming();
    sendOdom();
}

float piUartGetAngleDeg()
{
    return g_angleDeg;
}

float piUartGetMagnitude()
{
    return g_magnitude;
}

unsigned long piUartGetLastPacketMs()
{
    return g_lastPacketMs;
}

bool piUartIsStopped()
{
    return g_motorStopped;
}
