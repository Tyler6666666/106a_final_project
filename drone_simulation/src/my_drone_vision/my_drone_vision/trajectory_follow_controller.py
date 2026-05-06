#!/usr/bin/env python3
"""
State machine: SEARCH -> FOLLOW (record cmd_vel) -> WAIT -> REPLAY -> LAND.

Trajectory replay republishes the same Twist commands at the same 1/dt rate as during
FOLLOW (open-loop in the drone body frame).
"""
from __future__ import annotations

import time
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from tello_msgs.srv import TelloAction

TrajectorySample = Tuple[float, float, float, float]  # linear.x, linear.y, linear.z, angular.z


class TrajectoryFollowController(Node):
    """ArUco follower that records commanded velocities and replay them after a wait."""

    SEARCH = 0
    FOLLOW = 1
    WAIT = 2
    REPLAY = 3
    DONE = 4  # after land service call (terminal)

    def __init__(self):
        super().__init__("trajectory_follow_controller_node")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("tello_action_service", "/tello_action")
        self.declare_parameter("pose_topic", "/aruco/pose_3d")

        self.declare_parameter("auto_takeoff", True)
        self.declare_parameter("follow_after_takeoff_sec", 2.0)
        self.declare_parameter("target_dist", 0.60)
        self.declare_parameter("require_tag_before_takeoff", True)
        self.declare_parameter("tag_confirm_sec", 5.0)

        self.declare_parameter("memory_sec", 30.0)
        self.declare_parameter("wait_before_replay_sec", 3.0)

        self.declare_parameter("follow_land_on_tag_loss", False)
        self.declare_parameter("tag_loss_land_sec", 5.0)

        self.declare_parameter("enable_search", False)
        self.declare_parameter("search_speed", 0.25)
        self.declare_parameter("search_duration", 8.0)

        self.declare_parameter("max_forward_speed", 0.18)
        self.declare_parameter("max_side_speed", 0.12)
        self.declare_parameter("max_vertical_speed", 0.12)
        self.declare_parameter("max_yaw_speed", 0.25)
        self.declare_parameter("kp_side", 0.25)
        self.declare_parameter("kd_side", 0.03)
        self.declare_parameter("kp_yaw", 0.80)
        self.declare_parameter("kd_yaw", 0.05)
        self.declare_parameter("kp_fwd", 0.45)
        self.declare_parameter("kd_fwd", 0.08)
        self.declare_parameter("kp_z", 0.55)
        self.declare_parameter("yaw_from_x_gain", 0.75)
        self.declare_parameter("anticipate_gain", 0.0)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        tello_action_service = self.get_parameter("tello_action_service").value
        pose_topic = self.get_parameter("pose_topic").value

        self.auto_takeoff_enabled = bool(self.get_parameter("auto_takeoff").value)
        self.follow_after_takeoff_sec = float(self.get_parameter("follow_after_takeoff_sec").value)
        self.target_dist = float(self.get_parameter("target_dist").value)
        self.require_tag_before_takeoff = bool(self.get_parameter("require_tag_before_takeoff").value)
        self.tag_confirm_sec = float(self.get_parameter("tag_confirm_sec").value)

        self.memory_sec = float(self.get_parameter("memory_sec").value)
        self.wait_before_replay_sec = float(self.get_parameter("wait_before_replay_sec").value)

        self.follow_land_on_tag_loss = bool(self.get_parameter("follow_land_on_tag_loss").value)
        self.tag_loss_land_sec = float(self.get_parameter("tag_loss_land_sec").value)

        self.enable_search = bool(self.get_parameter("enable_search").value)
        self.search_speed = float(self.get_parameter("search_speed").value)
        self.search_duration = float(self.get_parameter("search_duration").value)

        self.max_forward_speed = float(self.get_parameter("max_forward_speed").value)
        self.max_side_speed = float(self.get_parameter("max_side_speed").value)
        self.max_vertical_speed = float(self.get_parameter("max_vertical_speed").value)
        self.max_yaw_speed = float(self.get_parameter("max_yaw_speed").value)

        self.Kp_side = float(self.get_parameter("kp_side").value)
        self.Kd_side = float(self.get_parameter("kd_side").value)
        self.Kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.Kd_yaw = float(self.get_parameter("kd_yaw").value)
        self.Kp_fwd = float(self.get_parameter("kp_fwd").value)
        self.Kd_fwd = float(self.get_parameter("kd_fwd").value)
        self.Kp_z = float(self.get_parameter("kp_z").value)
        self.K_anticipate = float(self.get_parameter("anticipate_gain").value)
        self.yaw_from_x_gain = float(self.get_parameter("yaw_from_x_gain").value)

        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.pose_sub = self.create_subscription(Pose, pose_topic, self.pose_cb, 10)
        self.takeoff_client = self.create_client(TelloAction, tello_action_service)

        self.dt = 0.05
        self.last_err_x = 0.0
        self.last_err_yaw = 0.0
        self.last_err_fwd = 0.0
        self.filter_alpha = 0.4
        self.smooth_dx = 0.0
        self.smooth_dyaw = 0.0
        self.smooth_dfwd = 0.0
        self.control_initialized = False

        self.new_pose_received = False
        self.last_cmd = Twist()
        self.latest_pose: Optional[Pose] = None
        self.last_seen = 0.0
        self.first_seen: Optional[float] = None
        self.is_searching = False
        self.search_start_time = 0.0
        self.search_cooldown = False
        self.waiting_for_tag = False

        self.start_time = time.time()
        self.has_sent_takeoff = False
        self.takeoff_sent_time: Optional[float] = None
        self.follow_mode_logged = False
        self.follow_mode_started = False
        self.has_sent_land = False

        self.mission_state = self.SEARCH
        self.follow_start_time: Optional[float] = None
        self.wait_start_time: Optional[float] = None
        self.replay_index = 0
        self.trajectory: List[TrajectorySample] = []

        self.create_timer(self.dt, self.control_loop)
        self.create_timer(1.0, self.auto_takeoff)

        self.get_logger().info(
            f"Trajectory follow: memory_sec={self.memory_sec}, wait_before_replay_sec={self.wait_before_replay_sec}"
        )

    def pose_cb(self, msg: Pose) -> None:
        self.latest_pose = msg
        if self.first_seen is None:
            self.first_seen = time.time()
        self.last_seen = time.time()
        self.new_pose_received = True
        if self.mission_state <= self.FOLLOW:
            self.is_searching = False
            self.search_cooldown = False
            self.waiting_for_tag = False

    def control_loop(self) -> None:
        cmd = Twist()
        now = time.time()

        if self.mission_state == self.DONE or self.has_sent_land:
            self.vel_pub.publish(cmd)
            return

        if self.mission_state == self.REPLAY:
            self._replay_step(cmd)
            return

        if self.mission_state == self.WAIT:
            self._wait_step(cmd, now)
            return

        # SEARCH and FOLLOW
        if self.auto_takeoff_enabled and not self.has_sent_takeoff:
            self._search_step(cmd, now)
            return

        if self.has_sent_takeoff and self.takeoff_sent_time is not None:
            elapsed_since_takeoff = now - self.takeoff_sent_time
            if elapsed_since_takeoff < self.follow_after_takeoff_sec:
                self.vel_pub.publish(cmd)
                return
            if not self.follow_mode_logged:
                self.get_logger().warn(
                    f"Entering FOLLOW (record) {elapsed_since_takeoff:.1f}s after takeoff command."
                )
                self.follow_mode_logged = True
                self.follow_mode_started = True
                self.reset_controller_state()
                self.mission_state = self.FOLLOW
                self.follow_start_time = now
                self.trajectory.clear()

        if not self.auto_takeoff_enabled and now - self.start_time < 1.0:
            self.vel_pub.publish(cmd)
            return

        if self.mission_state == self.FOLLOW:
            if self.follow_start_time is not None and (now - self.follow_start_time) >= self.memory_sec:
                self.get_logger().warn(
                    f"FOLLOW complete: recorded {len(self.trajectory)} samples over {self.memory_sec:.1f}s."
                )
                self.mission_state = self.WAIT
                self.wait_start_time = now
                self.vel_pub.publish(cmd)
                return

        time_since_last_seen = now - self.last_seen

        if self.follow_land_on_tag_loss and self.should_land_on_tag_loss(now, time_since_last_seen):
            self.vel_pub.publish(cmd)
            self._enter_land("Tag lost during mission; landing.")
            return

        if self.latest_pose is not None and time_since_last_seen < 0.6:
            if self.new_pose_received:
                self.run_pd_control(cmd)
                self.last_cmd = cmd
                self.new_pose_received = False
            elif time_since_last_seen < 0.15:
                cmd = self._copy_twist(self.last_cmd)
            else:
                cmd = self._copy_twist(self.last_cmd)
                cmd.linear.x *= 0.95
                cmd.linear.y *= 0.95
                cmd.linear.z *= 0.95
                cmd.angular.z *= 0.95
            self.is_searching = False
        elif self.enable_search and time_since_last_seen > 1.0 and not self.search_cooldown:
            if not self.is_searching:
                self.get_logger().warn("Target lost; starting slow yaw search.")
                self.is_searching = True
                self.search_start_time = now
            if now - self.search_start_time < self.search_duration:
                cmd.angular.z = self.search_speed
            else:
                self.get_logger().warn("Search complete; staying idle.")
                self.is_searching = False
                self.search_cooldown = True
        else:
            if not self.waiting_for_tag and self.follow_mode_started:
                self.get_logger().warn("ArUco tag lost; hovering until tag returns.")
                self.waiting_for_tag = True
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0

        if self.mission_state == self.FOLLOW:
            self.trajectory.append(
                (float(cmd.linear.x), float(cmd.linear.y), float(cmd.linear.z), float(cmd.angular.z))
            )

        self.vel_pub.publish(cmd)

    def _search_step(self, cmd: Twist, now: float) -> None:
        """Pre-takeoff: optional yaw search only works after airborne; on ground we mostly wait for tag."""
        time_since_last_seen = now - self.last_seen
        if self.enable_search and self.has_sent_takeoff and time_since_last_seen > 1.0:
            if not self.is_searching:
                self.is_searching = True
                self.search_start_time = now
            if now - self.search_start_time < self.search_duration:
                cmd.angular.z = self.search_speed
        self.vel_pub.publish(cmd)

    def _wait_step(self, cmd: Twist, now: float) -> None:
        if self.wait_start_time is None:
            self.wait_start_time = now
        if (now - self.wait_start_time) >= self.wait_before_replay_sec:
            self.get_logger().warn("WAIT complete; starting REPLAY.")
            if len(self.trajectory) == 0:
                self.get_logger().warn("Empty trajectory; skipping REPLAY.")
                self._enter_land("Empty trajectory.")
                return
            self.mission_state = self.REPLAY
            self.replay_index = 0
            return
        self.vel_pub.publish(cmd)

    def _replay_step(self, cmd: Twist) -> None:
        if self.replay_index >= len(self.trajectory):
            self._enter_land("REPLAY complete.")
            return
        lx, ly, lz, az = self.trajectory[self.replay_index]
        cmd.linear.x = lx
        cmd.linear.y = ly
        cmd.linear.z = lz
        cmd.angular.z = az
        self.replay_index += 1
        self.vel_pub.publish(cmd)

    def _enter_land(self, reason: str) -> None:
        self.get_logger().warn(reason)
        self.send_land_once()

    def should_land_on_tag_loss(self, now: float, time_since_last_seen: float) -> bool:
        if not self.follow_land_on_tag_loss or self.has_sent_land:
            return False
        if self.auto_takeoff_enabled and not self.follow_mode_started:
            return False
        if self.latest_pose is None:
            if self.auto_takeoff_enabled and self.takeoff_sent_time is not None:
                return now - self.takeoff_sent_time >= self.follow_after_takeoff_sec + self.tag_loss_land_sec
            return now - self.start_time >= self.tag_loss_land_sec
        return time_since_last_seen >= self.tag_loss_land_sec

    def send_land_once(self) -> None:
        if self.has_sent_land:
            self.mission_state = self.DONE
            return
        self.has_sent_land = True
        if not self.takeoff_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().error("Cannot land: tello_action service unavailable.")
            self.mission_state = self.DONE
            return
        req = TelloAction.Request()
        req.cmd = "land"
        self.takeoff_client.call_async(req)
        self.mission_state = self.DONE

    def reset_controller_state(self) -> None:
        self.last_err_x = 0.0
        self.last_err_yaw = 0.0
        self.last_err_fwd = 0.0
        self.smooth_dx = 0.0
        self.smooth_dyaw = 0.0
        self.smooth_dfwd = 0.0
        self.last_cmd = Twist()
        self.control_initialized = False

    def run_pd_control(self, cmd: Twist) -> None:
        assert self.latest_pose is not None
        err_x = -self.latest_pose.position.x
        err_yaw = -self.latest_pose.orientation.z
        err_fwd = self.latest_pose.position.z - self.target_dist

        active_err_x = err_x + (self.latest_pose.orientation.z * self.K_anticipate)

        if not self.control_initialized:
            self.last_err_x = active_err_x
            self.last_err_yaw = err_yaw
            self.last_err_fwd = err_fwd
            self.control_initialized = True

        raw_dx = (active_err_x - self.last_err_x) / self.dt
        raw_dyaw = (err_yaw - self.last_err_yaw) / self.dt
        raw_dfwd = (err_fwd - self.last_err_fwd) / self.dt

        self.last_err_x = active_err_x
        self.last_err_yaw = err_yaw
        self.last_err_fwd = err_fwd

        self.smooth_dx = self.filter_alpha * raw_dx + (1 - self.filter_alpha) * self.smooth_dx
        self.smooth_dyaw = self.filter_alpha * raw_dyaw + (1 - self.filter_alpha) * self.smooth_dyaw
        self.smooth_dfwd = self.filter_alpha * raw_dfwd + (1 - self.filter_alpha) * self.smooth_dfwd

        v_yaw = (err_yaw * self.Kp_yaw) + (self.smooth_dyaw * self.Kd_yaw) + (err_x * self.yaw_from_x_gain)
        vy = (active_err_x * self.Kp_side) + (self.smooth_dx * self.Kd_side)
        vx = (err_fwd * self.Kp_fwd) + (self.smooth_dfwd * self.Kd_fwd)
        vz = -self.latest_pose.position.y * self.Kp_z

        def smooth_deadzone(val: float, threshold: float) -> float:
            return 0.0 if abs(val) < threshold else val

        cmd.angular.z = float(np.clip(smooth_deadzone(v_yaw, 0.06), -self.max_yaw_speed, self.max_yaw_speed))
        cmd.linear.x = float(np.clip(smooth_deadzone(vx, 0.04), -self.max_forward_speed, self.max_forward_speed))
        cmd.linear.y = float(np.clip(smooth_deadzone(vy, 0.04), -self.max_side_speed, self.max_side_speed))
        cmd.linear.z = float(np.clip(smooth_deadzone(vz, 0.04), -self.max_vertical_speed, self.max_vertical_speed))

    def auto_takeoff(self) -> None:
        if not self.auto_takeoff_enabled or self.has_sent_takeoff:
            return
        now = time.time()
        if now - self.start_time <= 4.0:
            return
        if self.require_tag_before_takeoff:
            if now - self.last_seen > 0.75:
                self.first_seen = None
                return
            if self.first_seen is None or now - self.first_seen < self.tag_confirm_sec:
                return
        if not self.takeoff_client.wait_for_service(timeout_sec=1.0):
            return

        req = TelloAction.Request()
        req.cmd = "takeoff"
        self.takeoff_client.call_async(req)
        self.has_sent_takeoff = True
        self.takeoff_sent_time = time.time()
        self.get_logger().warn(
            f"Takeoff requested; FOLLOW begins {self.follow_after_takeoff_sec:.1f}s after takeoff cmd."
        )

    @staticmethod
    def _copy_twist(src: Twist) -> Twist:
        dst = Twist()
        dst.linear.x = src.linear.x
        dst.linear.y = src.linear.y
        dst.linear.z = src.linear.z
        dst.angular.x = src.angular.x
        dst.angular.y = src.angular.y
        dst.angular.z = src.angular.z
        return dst

    def stop_motion(self) -> None:
        try:
            if rclpy.ok():
                self.vel_pub.publish(Twist())
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryFollowController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motion()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
