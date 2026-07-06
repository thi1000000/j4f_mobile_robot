#!/bin/bash
# Run once on Raspberry Pi: bash install_on_pi.sh
set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"
LOG_DIR="${LOG_DIR:-$HOME/logs}"
BIN_DIR="$HOME/bin"

echo "=== Install robot web stack ==="
mkdir -p "$LOG_DIR" "$BIN_DIR"

pip3 install --user -r requirements.txt

chmod +x "$SCRIPT_DIR"/*.sh

for script in start_web.sh start_all.sh start_slam.sh start_rosbridge.sh start_tf.sh; do
  ln -sf "$SCRIPT_DIR/$script" "$BIN_DIR/$script"
done

echo "=== Start web server ==="
nohup "$SCRIPT_DIR/start_web.sh" >/dev/null 2>&1 &
sleep 2

if ss -tlnp 2>/dev/null | grep -q ':8080 '; then
  IP=$(hostname -I | awk '{print $1}')
  echo "OK: http://${IP}:8080/"
else
  echo "WARN: port 8080 not open yet. Check: tail -50 $LOG_DIR/web.log"
  tail -20 "$LOG_DIR/web.log" 2>/dev/null || true
  exit 1
fi
