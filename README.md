# Drone Simulation Setup

This repository contains a ROS 2 Humble + Gazebo Classic simulation setup for:

- a simulated Tello drone,
- ArUco-based visual tracking,
- a custom movable `aruco_marker` Gazebo model,
- and a custom 3D Gazebo plugin that accepts `cmd_vel` with `x / y / z / yaw`.

## Tested Environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- Python 3.10

## 1. Clone the Repository

```bash
mkdir -p ~/ros2_ws/106a_final_project
cd ~/ros2_ws/106a_final_project
git clone <your-repo-url> drone_simulation
cd drone_simulation
```

## 2. Install System Dependencies

Install ROS 2 Humble first, then install the packages used by this project:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-cv-bridge \
  ros-humble-vision-msgs \
  ros-humble-image-transport \
  ros-humble-camera-info-manager \
  python3-opencv \
  python3-colcon-common-extensions
```

If your machine already has ROS 2 Humble, you only need the missing packages.

## 3. Build the Workspace

```bash
cd ~/ros2_ws/106a_final_project/drone_simulation
source /opt/ros/humble/setup.bash
colcon build
```

## 4. Source the Environment

Open a new terminal for every run and source both ROS and the workspace:

```bash
cd ~/ros2_ws/106a_final_project/drone_simulation
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 5. Make the Custom Gazebo Model Available

The repository now tracks the custom model here:

```bash
src/tello_ros/tello_gazebo/models/aruco_marker
```

You have two ways to use it on a new computer.

### Option A: Recommended

Use the repository model path directly:

```bash
export GAZEBO_MODEL_PATH=$PWD/install/tello_gazebo/share/tello_gazebo/models:$GAZEBO_MODEL_PATH
```

This lets Gazebo find `aruco_marker` without copying anything into `~/.gazebo/models`.

### Option B: GUI Insert Convenience

If you want the model to appear in Gazebo's local model list for manual insertion:

```bash
mkdir -p ~/.gazebo/models
cp -r src/tello_ros/tello_gazebo/models/aruco_marker ~/.gazebo/models/
```

After that, restart Gazebo before testing.

## 6. Run the Drone Simulation

Start the simulated drone:

```bash
cd ~/ros2_ws/106a_final_project/drone_simulation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch tello_gazebo simple_launch.py
```

`simple_launch.py` also injects the repository model directory into `GAZEBO_MODEL_PATH`.

## 7. Run the Vision Nodes

In a second terminal:

```bash
cd ~/ros2_ws/106a_final_project/drone_simulation
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_drone_vision vision_control.launch.py
```

## 8. Run the Moving ArUco Marker

In another terminal:

```bash
cd ~/ros2_ws/106a_final_project/drone_simulation
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 marker_move.py
```

The current marker motion script publishes a spatial figure-eight trajectory to:

```bash
/target_marker/cmd_vel
```

The custom Gazebo plugin `libgazebo_ros_3d_move.so` makes the marker respond to:

- `linear.x`
- `linear.y`
- `linear.z`
- `angular.z`

It also clamps the marker within a safe altitude range so it does not drop to the ground.

## 9. Manual Gazebo Insertion Notes

If you manually insert `aruco_marker` in Gazebo:

- restart Gazebo after changing the model files,
- make sure the repository or `~/.gazebo/models` contains the updated `aruco_marker`,
- and ensure you are using the tracked model from this repository, not an outdated local copy.

## 10. Common Troubleshooting

### Gazebo says `Address already in use`

Another Gazebo instance is still running:

```bash
pkill -f "gazebo|gzserver|gzclient"
```

Then launch again.

### The marker does not move in `z`

Make sure Gazebo is loading the repository version of `aruco_marker`, which uses:

```bash
libgazebo_ros_3d_move.so
```

If Gazebo is still using an older model that references `libgazebo_ros_planar_move.so`, the marker will only move in the plane.

### The marker is not detected

Check:

- the drone camera topic is active,
- the inserted model is really `aruco_marker`,
- Gazebo was restarted after model changes,
- and the vision nodes were started after sourcing `install/setup.bash`.
