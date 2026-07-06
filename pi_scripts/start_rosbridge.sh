#!/bin/bash
# WebSocket bridge for browser SLAM map (port 9090)
set -e
source /opt/ros/jazzy/setup.bash
exec ros2 launch rosbridge_server rosbridge_websocket_launch.xml
