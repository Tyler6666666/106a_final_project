# Real Tello Drone ArUco Follow

这个分支用于真实 DJI/Ryze Tello 无人机的 ArUco tag 跟随测试。系统使用 `djitellopy` 直接连接 Tello，相机画面直接在本机做 ArUco 检测，再通过 `/cmd_vel` 转成 Tello RC 控制指令。

## 功能

- 直接读取 Tello 相机视频，降低 ROS 图像链路延迟。
- 检测 ArUco `DICT_6X6_50` 中 ID 为 `0` 的 marker。
- 起飞前需要连续看到 tag 约 5 秒。
- 起飞后等待 2 秒进入跟随模式。
- 跟随目标距离约 0.60 m。
- 跟随过程中丢失 tag 超过 5 秒自动降落。
- 支持手动降落和 emergency 急停。

## 准备

先确认已经连接到 Tello 的 Wi-Fi。Tello 默认网络名一般类似：

```bash
TELLO-XXXXXX
```

进入工作区：

```bash
cd ~/ros2_ws/106a_final_project/drone_simulation
```

确认当前分支：

```bash
git branch --show-current
```

应该看到：

```bash
real-tello-drone-follow
```

安装 Python 依赖：

```bash
python3 -m pip install --user djitellopy
```

不要手动升级 `numpy` 或 `opencv-python`，否则可能影响 ROS Humble 自带的 OpenCV 环境。

## 编译

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select my_drone_vision
source install/setup.bash
```

## 启动真实跟随

先把 ArUco tag 放在 Tello 前方，让相机能稳定看到它，然后运行：

```bash
source install/setup.bash
ros2 launch my_drone_vision real_follow.launch.py
```

正常流程：

1. 程序连接 Tello。
2. 打开视频流。
3. 检测到 ArUco tag 并稳定看到约 5 秒。
4. 自动发送 `takeoff`。
5. 起飞后等待 2 秒。
6. 进入跟随模式。
7. 如果丢失 tag 超过 5 秒，自动发送 `land`。

## 手动降落

如果需要马上降落，另开一个 terminal：

```bash
cd ~/ros2_ws/106a_final_project/drone_simulation
source install/setup.bash
ros2 service call /tello_action tello_msgs/srv/TelloAction "{cmd: 'land'}"
```

降落后建议停止 launch 进程，释放 Tello 视频流。

## Emergency 急停

只有在危险情况下使用 emergency。这个命令会立即停桨：

```bash
cd ~/ros2_ws/106a_final_project/drone_simulation
source install/setup.bash
ros2 service call /tello_action tello_msgs/srv/TelloAction "{cmd: 'emergency'}"
```

## 视觉检查

只看是否有 ArUco pose 输出：

```bash
source install/setup.bash
ros2 topic echo /aruco/pose_3d --once
```

只看 controller 是否在发速度：

```bash
source install/setup.bash
ros2 topic echo /cmd_vel --once
```

查看当前 Tello 相关进程：

```bash
pgrep -af "real_follow.launch.py|tello_direct_io|aruco_controller"
```

## 主要文件

- `src/my_drone_vision/launch/real_follow.launch.py`
  - 真实 Tello 跟随启动文件。
  - 当前配置为丢失 tag 5 秒自动降落。
- `src/my_drone_vision/my_drone_vision/tello_direct_io.py`
  - 直接连接 Tello、读取视频、检测 ArUco、发送 RC 控制。
- `src/my_drone_vision/my_drone_vision/aruco_controller.py`
  - 根据 ArUco pose 计算跟随速度，并负责自动起飞和丢失目标降落。

## 关键参数

在 `real_follow.launch.py` 中可以调整：

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

RC 输出限制在 `tello_direct_io` 参数中：

```python
'max_lr_rc': 12,
'max_fb_rc': 18,
'max_ud_rc': 12,
'max_yaw_rc': 25,
'rc_send_period_sec': 0.10,
```

这些值比较保守，适合真实无人机初次测试。

## 安全建议

- 第一次测试时不要靠近人、墙、桌子或玻璃。
- 电池尽量保持在 30% 以上。
- tag 要放在 Tello 正前方，距离约 0.5 到 0.8 m。
- 测试时准备好手动 `land` 命令。
- 如果画面卡住或控制异常，先手动降落，再停止 launch。
- 不要同时运行 `tello_driver` 和 `tello_direct_io`，它们会抢同一个 Tello 连接。

## 常见问题

如果无法连接 Tello：

```bash
ping 192.168.10.1
```

如果没有 ArUco 输出：

- 确认 tag ID 是 `0`。
- 确认 tag 字典是 `DICT_6X6_50`。
- 确认光线足够，tag 没有严重反光。
- 确认 debug 窗口中能看到完整 tag。

如果起飞后没有跟随：

- 系统会先等待 2 秒再进入跟随。
- 如果 tag 丢失超过 5 秒，会自动降落。
- 检查 `/aruco/pose_3d` 是否还在更新。

如果测试结束后 Tello 视频流被占用：

```bash
pgrep -af "real_follow.launch.py|tello_direct_io|aruco_controller"
```

确认进程后再结束对应 PID。
