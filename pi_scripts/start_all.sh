#!/bin/bash
set -e

LOG_DIR=/home/pikii/logs
mkdir -p "$LOG_DIR"
START_TS=$(date +%s)

log() { echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_DIR/boot.log"; }

log "=== Boot start ==="

pkill -f 'ros2 launch rplidar_ros' 2>/dev/null || true
pkill -f rplidar_node 2>/dev/null || true
pkill -f 'ros2 launch slam_toolbox' 2>/dev/null || true
pkill -f async_slam_toolbox 2>/dev/null || true
pkill -f sync_slam_toolbox 2>/dev/null || true
pkill -f static_transform_publisher 2>/dev/null || true
pkill -f rosbridge_websocket 2>/dev/null || true
pkill -f 'http.server 8080' 2>/dev/null || true
pkill -f robot_bridge.py 2>/dev/null || true
sleep 1

for i in $(seq 1 45); do
  if [ -e /dev/ttyUSB0 ]; then
    log "Lidar USB ready (${i}s)"
    break
  fi
  sleep 1
done

if [ ! -e /dev/ttyUSB0 ]; then
  log "WARN: /dev/ttyUSB0 chua co - thu lai sau 30s"
  (
    sleep 30
    if [ -e /dev/ttyUSB0 ] && ! pgrep -f rplidar_node >/dev/null; then
      echo "[$(date '+%Y-%m-%d %H:%M:%S')] Retry lidar (USB cap muon)" >> "$LOG_DIR/boot.log"
      nohup /home/pikii/start_rplidar_c1.sh >> "$LOG_DIR/rplidar.log" 2>&1 &
    fi
  ) &
fi

log "Starting lidar..."
nohup /home/pikii/start_rplidar_c1.sh >> "$LOG_DIR/rplidar.log" 2>&1 &
sleep 2

log "Starting SLAM..."
nohup /home/pikii/start_slam.sh >> "$LOG_DIR/slam.log" 2>&1 &
sleep 2

log "Starting rosbridge + web (robot bridge)..."
nohup /home/pikii/start_rosbridge.sh >> "$LOG_DIR/rosbridge.log" 2>&1 &
nohup /home/pikii/start_web.sh >> "$LOG_DIR/web.log" 2>&1 &
sleep 1

ELAPSED=$(( $(date +%s) - START_TS ))
log "=== Boot complete (${ELAPSED}s) ==="
log "Control + SLAM: http://$(hostname -I | awk '{print $1}'):8080/"
