#!/bin/bash
# Web + UART bridge (joystick control + SLAM map on :8080)
cd "$(dirname "$0")"
export ROBOT_UART="${ROBOT_UART:-/dev/ttyS0}"
exec python3 robot_bridge.py
