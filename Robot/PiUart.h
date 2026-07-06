#ifndef PI_UART_H
#define PI_UART_H

#include <Arduino.h>

#define PI_UART_RX 18
#define PI_UART_TX 19
#define PI_UART_BAUD 115200

bool piUartBegin();

void piUartUpdate();

float piUartGetAngleDeg();

float piUartGetMagnitude();

unsigned long piUartGetLastPacketMs();

bool piUartIsStopped();

#endif
