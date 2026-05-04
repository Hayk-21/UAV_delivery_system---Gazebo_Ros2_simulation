#!/bin/bash
# Launch PX4 SITL with city world, 4 drones, and MicroXRCE Agent.
# Usage: ./launch_city.sh
set -e

PX4_DIR="$HOME/PX4-Autopilot"
cd "$PX4_DIR"

echo "=== Cleaning up old processes ==="
pkill -9 px4 2>/dev/null || true
pkill -9 -f 'gz sim' 2>/dev/null || true
pkill -9 -f ruby 2>/dev/null || true
pkill -9 MicroXRCEAgent 2>/dev/null || true
rm -rf /tmp/px4* 2>/dev/null || true
sleep 2

echo "=== Starting MicroXRCE Agent (background) ==="
MicroXRCEAgent udp4 -p 8888 &
AGENT_PID=$!
echo "  Agent PID: $AGENT_PID"

echo "=== Building PX4 SITL ==="
# Build first (blocking)
make px4_sitl_default 2>&1 | tail -3

echo "=== Starting PX4 instance 1 + Gazebo (x500_1, spawns world) ==="
PX4_SIM_SPEED_FACTOR=1 PX4_GZ_WORLD=city PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_NAME=x500_1 \
  ./build/px4_sitl_default/bin/px4 -i 1 &
PX4_1_PID=$!
echo "  PX4 instance 1 PID: $PX4_1_PID"

echo "  Waiting for Gazebo to load..."
sleep 15

echo "=== Starting PX4 instance 2 (x500_2) ==="
PX4_SIM_SPEED_FACTOR=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_NAME=x500_2 \
  ./build/px4_sitl_default/bin/px4 -i 2 &
PX4_2_PID=$!
echo "  PX4 instance 2 PID: $PX4_2_PID"
sleep 10

echo "=== Starting PX4 instance 3 (x500_3) ==="
PX4_SIM_SPEED_FACTOR=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_NAME=x500_3 \
  ./build/px4_sitl_default/bin/px4 -i 3 &
PX4_3_PID=$!
echo "  PX4 instance 3 PID: $PX4_3_PID"

echo ""
echo "============================================"
echo "  All processes started!"
echo "  Agent:    PID $AGENT_PID"
echo "  Drone 1:  PID $PX4_1_PID"
echo "  Drone 2:  PID $PX4_2_PID"
echo "  Drone 3:  PID $PX4_3_PID"
echo ""
echo "  Now run the dashboard in another terminal:"
echo "    source /opt/ros/jazzy/setup.bash && source ~/ros2_ws/install/setup.bash"
echo "    python3 ~/PX4-Autopilot/scripts/drone_dashboard.py"
echo "============================================"
echo ""
echo "Press Ctrl+C to stop everything."

# Cleanup on exit
cleanup() {
    echo ""
    echo "=== Shutting down ==="
    kill $PX4_1_PID $PX4_2_PID $PX4_3_PID $AGENT_PID 2>/dev/null || true
    sleep 1
    pkill -9 px4 2>/dev/null || true
    pkill -9 -f 'gz sim' 2>/dev/null || true
    pkill -9 -f ruby 2>/dev/null || true
    pkill -9 MicroXRCEAgent 2>/dev/null || true
    rm -rf /tmp/px4* 2>/dev/null || true
    echo "Done."
}
trap cleanup EXIT INT TERM

wait
