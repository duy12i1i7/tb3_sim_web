#!/usr/bin/env bash
set -e

# ROS env
source /opt/ros/${ROS_DISTRO}/setup.bash
source /opt/ros2_ws/install/setup.bash

# Avoid warnings
export XDG_RUNTIME_DIR=/tmp/runtime-$(id -u)
mkdir -p "$XDG_RUNTIME_DIR"
chmod 700 "$XDG_RUNTIME_DIR"

# noVNC display
export DISPLAY=:1
export USER=${USER:-ubuntu}

# Make sure Gazebo Sim sees models (model://...)
export GZ_SIM_RESOURCE_PATH=${GZ_SIM_RESOURCE_PATH:-/opt/gz_models}

# Start virtual X server + desktop + VNC + noVNC
Xvfb :1 -screen 0 1600x900x24 -ac +extension GLX +render -noreset &
sleep 1
startxfce4 &

# VNC server (no password for quick local use; if you want a password, tell me)
x11vnc -display :1 -forever -shared -rfbport 5900 -nopw -quiet &
# noVNC web (http://localhost:6080/vnc.html)
websockify --web=/usr/share/novnc/ 6080 localhost:5900 --heartbeat=30 &

exec "$@"

