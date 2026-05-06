# Real Tello Drone ArUco Follow

This branch is for testing ArUco tag following on a real DJI/Ryze Tello drone. It uses `djitellopy` to connect directly to the Tello, reads the camera stream locally, detects the ArUco tag, and converts `/cmd_vel` commands into Tello RC control commands.

## Features

- Reads the Tello camera stream directly to reduce ROS image pipeline latency.
- Detects marker ID `0` from the ArUco `DICT_6X6_50` dictionary.
- Requires the tag to be visible for about 5 seconds before takeoff.
- Waits 2 seconds after takeoff before entering follow mode.
- Follows the tag at about 0.60 m.
- Automatically lands if the tag is lost for more than 5 seconds during follow mode.
- Supports manual landing and emergency motor stop.
- Adds real-drone face tracking using OpenCV Haar cascade detection.
- Face tracking waits for a stable face, takes off automatically, keeps the face near image center, follows at about 1 m, and lands after target loss.

## Setup

First connect your computer to the Tello Wi-Fi network. The default network name usually looks like:

```bash
TELLO-XXXXXX
```

Enter the ROS 2 workspace:

```bash
cd ~/ros2_ws/106a_final_project/drone_simulation
```

Check the current branch:

```bash
git branch --show-current
```

You should see:

```bash
real-tello-drone-follow
```

Install the Python dependency:

```bash
python3 -m pip install --user djitellopy
```

Do not manually upgrade `numpy` or `opencv-python`, because that can break the OpenCV version provided by ROS Humble.

## Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select my_drone_vision
source install/setup.bash
```

## Start Real Drone Follow Mode

Place the ArUco tag in front of the Tello so the camera can see it clearly, then run:

```bash
source install/setup.bash
ros2 launch my_drone_vision real_follow.launch.py
```

Expected flow:

1. The program connects to the Tello.
2. The video stream starts.
3. The ArUco tag is detected and remains visible for about 5 seconds.
4. The system sends `takeoff`.
5. The drone waits 2 seconds after takeoff.
6. Follow mode starts.
7. If the tag is lost for more than 5 seconds, the system sends `land`.

## Start Face Tracking Mode

Connect to the Tello Wi-Fi network, stand in front of the camera, then run:

```bash
source install/setup.bash
ros2 launch my_drone_vision face_tracking.launch.py
```

Expected flow:

1. `face_detector` connects to the Tello, reads video, detects the largest face, publishes `/face/pose`, accepts `/cmd_vel`, and exposes `/tello_action`.
2. `face_controller` waits until a face is stable for about 2 seconds.
3. The system sends `takeoff`.
4. The drone waits 2 seconds after takeoff.
5. Face follow mode starts.
6. The controller tries to keep the face centered in the image and maintain about 1 m distance using face area as the distance estimate.
7. If the face is lost for more than 5 seconds, the system sends `land`.

## Manual Landing

To land immediately, open another terminal:

```bash
cd ~/ros2_ws/106a_final_project/drone_simulation
source install/setup.bash
ros2 service call /tello_action tello_msgs/srv/TelloAction "{cmd: 'land'}"
```

After landing, stop the launch process to release the Tello video stream.

## Emergency Stop

Use emergency only in dangerous situations. This command immediately stops the motors:

```bash
cd ~/ros2_ws/106a_final_project/drone_simulation
source install/setup.bash
ros2 service call /tello_action tello_msgs/srv/TelloAction "{cmd: 'emergency'}"
```

## Vision Checks

Check whether ArUco poses are being published:

```bash
source install/setup.bash
ros2 topic echo /aruco/pose_3d --once
```

Check whether face poses are being published:

```bash
source install/setup.bash
ros2 topic echo /face/pose --once
```

Check whether the controller is publishing velocity commands:

```bash
source install/setup.bash
ros2 topic echo /cmd_vel --once
```

Check current Tello-related processes:

```bash
pgrep -af "real_follow.launch.py|tello_direct_io|aruco_controller"
```

## Main Files

- `src/my_drone_vision/launch/real_follow.launch.py`
  - Launch file for real Tello following.
  - Currently configured to land after 5 seconds of tag loss.
- `src/my_drone_vision/my_drone_vision/tello_direct_io.py`
  - Connects directly to the Tello, reads video, detects ArUco, and sends RC control commands.
- `src/my_drone_vision/my_drone_vision/aruco_controller.py`
  - Computes follow velocity from the ArUco pose and handles automatic takeoff and tag-loss landing.
- `src/my_drone_vision/launch/face_tracking.launch.py`
  - Launch file for real Tello face tracking.
- `src/my_drone_vision/my_drone_vision/face_detector.py`
  - Connects directly to the Tello, reads video, detects the largest face, publishes `/face/pose`, and translates `/cmd_vel` into Tello RC commands.
- `src/my_drone_vision/my_drone_vision/face_controller.py`
  - Handles stable-face takeoff, face-centered tracking, approximate 1 m distance control, and face-loss landing.

## Key Parameters

These parameters can be adjusted in `real_follow.launch.py`:

```python
'target_dist': 0.60,
'land_on_tag_loss': True,
'tag_loss_land_sec': 5.0,
'require_tag_before_takeoff': True,
'tag_confirm_sec': 5.0,
'max_forward_speed': 0.18,
'max_side_speed': 0.12,
'max_vertical_speed': 0.12,
'max_yaw_speed': 0.25,
```

RC output limits are configured in the `tello_direct_io` launch parameters:

```python
'max_lr_rc': 12,
'max_fb_rc': 18,
'max_ud_rc': 12,
'max_yaw_rc': 25,
'rc_send_period_sec': 0.10,
```

These values are conservative and are intended for initial real-drone testing.

Face tracking parameters can be adjusted in `face_tracking.launch.py`:

```python
'target_face_area_ratio': 0.032,
'forward_area_threshold': 0.030,
'min_forward_speed': 0.14,
'max_forward_speed': 0.35,
'max_vertical_speed': 0.18,
'max_yaw_speed': 0.55,
'kp_forward': 1.35,
'kp_vertical': 0.70,
'kp_yaw': 1.25,
```

`target_face_area_ratio` is the main distance tuning value. Smaller values keep the drone farther away, and larger values bring it closer. The current value is tuned to approximate 1 m with a normal frontal face in the Tello camera.

## Safety Notes

- Do the first tests away from people, walls, desks, windows, and glass.
- Keep the battery above 30% when possible.
- Place the tag in front of the Tello, about 0.5 to 0.8 m away.
- Keep the manual `land` command ready while testing.
- If the video freezes or the control behavior looks wrong, land first, then stop the launch process.
- Do not run `tello_driver` and `tello_direct_io` at the same time, because they will compete for the same Tello connection.

## Troubleshooting

If the Tello cannot be reached:

```bash
ping 192.168.10.1
```

If no ArUco pose is published:

- Make sure the tag ID is `0`.
- Make sure the dictionary is `DICT_6X6_50`.
- Make sure the lighting is good and the tag is not strongly reflective.
- Make sure the full tag is visible in the debug window.

If the drone takes off but does not follow:

- The system waits 2 seconds after takeoff before follow mode starts.
- If the tag is lost for more than 5 seconds, the drone will land automatically.
- Check whether `/aruco/pose_3d` is still updating.

If the Tello video stream is still occupied after testing:

```bash
pgrep -af "real_follow.launch.py|tello_direct_io|aruco_controller"
```

Confirm the process IDs, then stop the related processes.
