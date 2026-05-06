# Real Tello Drone ArUco Follow

This branch is for testing ArUco tag following on a real DJI/Ryze Tello drone. It uses `djitellopy` to connect directly to the Tello, reads the camera stream locally, detects the ArUco tag, and converts `/cmd_vel` commands into Tello RC control commands.

## Features

- Reads the Tello camera stream directly to reduce ROS image pipeline latency.
- Detects marker ID `0` from the ArUco `DICT_6X6_50` dictionary.
- Requires the tag to be visible for about 5 seconds before takeoff.
- Waits 2 seconds after takeoff before entering follow mode.
- Follows the tag at about 0.60 m.
- Automatically lands if the tag is lost for more than 5 seconds during follow mode.
- **Trajectory following mode** (`trajectory_following.launch.py`): after a stabilized tag sighting, auto takeoff, record `cmd_vel` while following for a configurable interval, hover, replay the recorded command sequence in open loop, then land.
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

Expected flow (defaults match `real_follow.launch.py`):

1. The program connects to the Tello.
2. The video stream starts.
3. The ArUco tag is detected and remains visible for about 5 seconds.
4. The system sends `takeoff`.
5. The drone waits 2 seconds after takeoff.
6. Follow mode starts.
7. If the tag is lost for more than 5 seconds, the system sends `land`.

## Trajectory Following Mode (record + replay)

Leader drone carries ArUco ID `0` (same as real follow); follower records commanded velocities during follow and later replays them without vision.

### Launch

Place the tag in front of the Tello so the camera can see it clearly, build and source the workspace, then run:

```bash
source install/setup.bash
ros2 launch my_drone_vision trajectory_following.launch.py
```

This launches the **same** `tello_direct_io` node stack as `real_follow.launch.py`, but swaps `aruco_controller` for **`trajectory_follow_controller`**, which implements the mission state machine below.

Expected flow:

1. Wait until the ArUco tag is stable (`tag_confirm_sec`), then `takeoff`.
2. Zero velocity for `follow_after_takeoff_sec` (same settling idea as `real_follow`).
3. Follow using the same ArUco PD idea as real follow (`target_dist`, gains, speed limits).
4. For `memory_sec` (e.g. 30 s), append each published `cmd_vel` sample on a fixed timer (`dt` in code, 0.05 s).
5. Hover for `wait_before_replay_sec` (e.g. 3 s) at zero command.
6. Replay the saved list in order at the same `dt` (open-loop; no tag needed).
7. Call `land` once via `TelloAction`, then stop.

### State machine vs `real_follow`

The controller mirrors `aruco_controller` / `real_follow.launch.py` through takeoff and the initial post-takeoff dwell, then adds timed record, wait, replay, and land.

| Phase | Behavior |
| --- | --- |
| **SEARCH (logical state 0)** | Same as `aruco_controller`: require the tag to be visible and stable for `tag_confirm_sec`, then send `takeoff`; then publish **zero velocity** for `follow_after_takeoff_sec` (takeoff stabilization). |
| **FOLLOW (state 1)** | Same ArUco PD follower as real follow (`target_dist`, `kp_*`, `max_*_speed`). At each control step (fixed `dt` in code, currently 0.05 s), append the published `cmd_vel` sample `(linear.x, linear.y, linear.z, angular.z)` to a list. Recording lasts **`memory_sec`** (default 30 s). |
| **WAIT (state 2)** | Publish zeros for **`wait_before_replay_sec`** (default 3 s). |
| **REPLAY (state 3)** | Replay the stored sequence **in order** at the **same dt** as during recording. No tag or vision is used (open-loop playback). |
| **Land (your state 4)** | Call `tello_msgs/srv/TelloAction` with `land` once, then enter an internal **DONE** state and keep publishing zero `cmd_vel`. |

**Tag loss during follow:** `real_follow` lands after prolonged tag loss (`land_on_tag_loss: True`). For trajectory mode, **`follow_land_on_tag_loss` defaults to `False`** so brief occlusions do not abort the experiment. Set it to `True` in the launch file if you want real-follow-style safety.

**Ground RC and “search spin before takeoff”:** In `tello_direct_io`, non-zero RC is blocked until the drone is airborne (`airborne == True` after `takeoff`). So commanding yaw or translation **before takeoff** does not actually move the Tello; the intended flow is **tag visible → takeoff**, like `real_follow`. Optional slow yaw **search** (`enable_search: True`) applies **after takeoff**, if the tag is lost during follow/search behavior (same spirit as optional search in `aruco_controller`).

### How trajectory replay works

`tello_direct_io` maps `geometry_msgs/Twist` into Tello RC in the **body frame**: `linear.x` forward/back, `linear.y` left/right, `linear.z` up/down, `angular.z` yaw rate.

During **FOLLOW**, the controller stores the **exact commands it publishes**, with a fixed time step between samples. During **REPLAY**, it republishes that sequence with the **same spacing**. That reproduces roughly the **motion pattern and timing** relative to body axes, not a globally registered path.

Because there is **no GPS/SLAM**, changing start pose or disturbances (wind, battery, friction) means the **spatial path will not exactly match** the original in world coordinates; yaw-heavy segments diverge more in space because body-forward keeps rotating. Higher fidelity would require storing vision-relative errors, odometry, or an external pose source. **Pixel-distance keeping** would mean changing the follower to regulate image-plane error instead of 3D pose + `target_dist` (not what this trajectory node adds).

### Trajectory parameters (`trajectory_following.launch.py`)

Tune these on the **`trajectory_follow_controller`** entry in that launch file:

- **`memory_sec`**: Duration (seconds) to follow the leader **while recording** commands.
- **`wait_before_replay_sec`**: Hover time (seconds) after recording before replay starts.
- **`target_dist`**, **`max_*_speed`**, **`kp_*`**, etc.: Same meaning as in `real_follow` / `aruco_controller` for the follow phase.

The control period **`dt`** used for both recording and replay is defined in `trajectory_follow_controller.py` (`self.dt = 0.05`). Change it there if you want a different rate (keep recording and replay using the same value).

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
pgrep -af "real_follow.launch.py|trajectory_following.launch.py|tello_direct_io|aruco_controller|trajectory_follow_controller"
```

## Main Files

- `src/my_drone_vision/launch/real_follow.launch.py`
  - Launch file for real Tello following.
  - Currently configured to land after 5 seconds of tag loss.
- `src/my_drone_vision/launch/trajectory_following.launch.py`
  - Same `tello_direct_io` parameters as real follow; runs `trajectory_follow_controller` for record → wait → replay → land.
- `src/my_drone_vision/my_drone_vision/trajectory_follow_controller.py`
  - ArUco follow with timed recording of `cmd_vel`, open-loop replay, and mission parameters (`memory_sec`, `wait_before_replay_sec`, etc.).
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
pgrep -af "real_follow.launch.py|trajectory_following.launch.py|tello_direct_io|aruco_controller|trajectory_follow_controller"
```

Confirm the process IDs, then stop the related processes.
