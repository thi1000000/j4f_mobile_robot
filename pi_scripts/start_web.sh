#!/bin/bash
# Web + UART bridge (joystick control + SLAM map on :8080)
set -e
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

LOG_DIR="${LOG_DIR:-$HOME/logs}"
mkdir -p "$LOG_DIR"

# Empty = auto-detect /dev/serial0, /dev/ttyAMA0, /dev/ttyS0
export ROBOT_UART="${ROBOT_UART:-}"
export ROBOT_HTTP_PORT="${ROBOT_HTTP_PORT:-8080}"

pkill -f '[p]ython3 robot_bridge.py' 2>/dev/null || true
sleep 0.5

echo "[$(date '+%Y-%m-%d %H:%M:%S')] Starting robot_bridge on :${ROBOT_HTTP_PORT}" >> "$LOG_DIR/web.log"
exec python3 "$SCRIPT_DIR/robot_bridge.py" >> "$LOG_DIR/web.log" 2>&1
