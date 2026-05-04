#!/usr/bin/env bash

source /ros2_ws/106a_final_project/drone_simulation/install/setup.bash

echo "=== TAKEOFF ==="
timeout 12 ros2 service call /tello_action tello_msgs/srv/TelloAction "{cmd: 'takeoff'}"

echo "=== FLIGHT_AFTER_TAKEOFF ==="
timeout 8 ros2 topic echo /flight_data --once

echo "=== HOVER_ZERO_CMD_8S ==="
for _ in 1 2 3 4 5 6 7 8; do
  ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" >/dev/null 2>&1 || true
  sleep 1
done

echo "=== FLIGHT_BEFORE_LAND ==="
timeout 8 ros2 topic echo /flight_data --once

echo "=== LAND ==="
timeout 12 ros2 service call /tello_action tello_msgs/srv/TelloAction "{cmd: 'land'}"

sleep 5

echo "=== FLIGHT_AFTER_LAND ==="
timeout 8 ros2 topic echo /flight_data --once
