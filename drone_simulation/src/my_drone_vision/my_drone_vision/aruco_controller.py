#!/usr/bin/env python3
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from tello_msgs.srv import TelloAction


class SmartTracker(Node):
    def __init__(self):
        super().__init__("aruco_controller_node")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("tello_action_service", "/tello_action")
        self.declare_parameter("auto_takeoff", False)
        self.declare_parameter("follow_after_takeoff_sec", 1.0)
        self.declare_parameter("target_dist", 0.45)
        self.declare_parameter("land_on_tag_loss", True)
        self.declare_parameter("tag_loss_land_sec", 1.0)
        self.declare_parameter("enable_search", False)
        self.declare_parameter("require_tag_before_takeoff", True)
        self.declare_parameter("tag_confirm_sec", 5.0)
        self.declare_parameter("max_forward_speed", 0.18)
        self.declare_parameter("max_side_speed", 0.12)
        self.declare_parameter("max_vertical_speed", 0.12)
        self.declare_parameter("min_vertical_speed", 0.08)
        self.declare_parameter("vertical_deadband", 0.015)
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
        self.auto_takeoff_enabled = self.get_parameter("auto_takeoff").value
        self.follow_after_takeoff_sec = float(self.get_parameter("follow_after_takeoff_sec").value)
        self.target_dist = float(self.get_parameter("target_dist").value)
        self.land_on_tag_loss = self.get_parameter("land_on_tag_loss").value
        self.tag_loss_land_sec = float(self.get_parameter("tag_loss_land_sec").value)
        self.enable_search = self.get_parameter("enable_search").value
        self.require_tag_before_takeoff = self.get_parameter("require_tag_before_takeoff").value
        self.tag_confirm_sec = float(self.get_parameter("tag_confirm_sec").value)
        self.max_forward_speed = float(self.get_parameter("max_forward_speed").value)
        self.max_side_speed = float(self.get_parameter("max_side_speed").value)
        self.max_vertical_speed = float(self.get_parameter("max_vertical_speed").value)
        self.min_vertical_speed = float(self.get_parameter("min_vertical_speed").value)
        self.vertical_deadband = float(self.get_parameter("vertical_deadband").value)
        self.max_yaw_speed = float(self.get_parameter("max_yaw_speed").value)

        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.pose_sub = self.create_subscription(Pose, "/aruco/pose_3d", self.pose_cb, 10)
        self.takeoff_client = self.create_client(TelloAction, tello_action_service)

        self.Kp_side = float(self.get_parameter("kp_side").value)
        self.Kd_side = float(self.get_parameter("kd_side").value)
        self.Kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.Kd_yaw = float(self.get_parameter("kd_yaw").value)
        self.Kp_fwd = float(self.get_parameter("kp_fwd").value)
        self.Kd_fwd = float(self.get_parameter("kd_fwd").value)
        self.Kp_z = float(self.get_parameter("kp_z").value)
        self.K_anticipate = float(self.get_parameter("anticipate_gain").value)
        self.yaw_from_x_gain = float(self.get_parameter("yaw_from_x_gain").value)

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
        self.latest_pose = None
        self.last_seen = 0.0
        self.first_seen = None
        self.is_searching = False
        self.search_start_time = 0.0
        self.search_speed = 0.25
        self.search_duration = 8.0
        self.search_cooldown = False
        self.waiting_for_tag = False

        self.start_time = time.time()
        self.has_sent_takeoff = False
        self.takeoff_sent_time = None
        self.follow_mode_logged = False
        self.follow_mode_started = False
        self.has_sent_land = False

        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)
        self.create_timer(1.0, self.auto_takeoff)

    def pose_cb(self, msg):
        self.latest_pose = msg
        if self.first_seen is None:
            self.first_seen = time.time()
        self.last_seen = time.time()
        self.new_pose_received = True
        self.is_searching = False
        self.search_cooldown = False
        self.waiting_for_tag = False

    def control_loop(self):
        cmd = Twist()
        now = time.time()

        if self.has_sent_land:
            self.vel_pub.publish(cmd)
            return

        if self.auto_takeoff_enabled and not self.has_sent_takeoff:
            return

        if self.has_sent_takeoff and self.takeoff_sent_time is not None:
            elapsed_since_takeoff = now - self.takeoff_sent_time
            if elapsed_since_takeoff < self.follow_after_takeoff_sec:
                self.vel_pub.publish(cmd)
                return
            if not self.follow_mode_logged:
                self.get_logger().warn(
                    f"Entering ArUco follow mode {elapsed_since_takeoff:.1f}s after takeoff command."
                )
                self.follow_mode_logged = True
                self.follow_mode_started = True
                self.reset_controller_state()

        if not self.auto_takeoff_enabled and now - self.start_time < 1.0:
            self.vel_pub.publish(cmd)
            return

        time_since_last_seen = now - self.last_seen

        if self.should_land_on_tag_loss(now, time_since_last_seen):
            self.vel_pub.publish(cmd)
            self.send_land_once("ArUco tag lost; landing.")
            return

        if self.latest_pose is not None and time_since_last_seen < 0.6:
            if self.new_pose_received:
                self.run_pd_control(cmd)
                self.last_cmd = cmd
                self.new_pose_received = False
            elif time_since_last_seen < 0.15:
                cmd = self.last_cmd
            else:
                cmd.linear.x = self.last_cmd.linear.x * 0.95
                cmd.linear.y = self.last_cmd.linear.y * 0.95
                cmd.linear.z = self.last_cmd.linear.z * 0.95
                cmd.angular.z = self.last_cmd.angular.z * 0.95
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

        self.vel_pub.publish(cmd)

    def should_land_on_tag_loss(self, now, time_since_last_seen):
        if not self.land_on_tag_loss or self.has_sent_land:
            return False
        if self.auto_takeoff_enabled and not self.follow_mode_started:
            return False
        if self.latest_pose is None:
            if self.auto_takeoff_enabled and self.takeoff_sent_time is not None:
                return now - self.takeoff_sent_time >= self.follow_after_takeoff_sec + self.tag_loss_land_sec
            return now - self.start_time >= self.tag_loss_land_sec
        return time_since_last_seen >= self.tag_loss_land_sec

    def send_land_once(self, reason):
        if self.has_sent_land:
            return
        self.has_sent_land = True
        self.get_logger().warn(reason)
        if not self.takeoff_client.wait_for_service(timeout_sec=0.2):
            self.get_logger().error("Cannot land: /tello_action service unavailable.")
            return
        req = TelloAction.Request()
        req.cmd = "land"
        self.takeoff_client.call_async(req)

    def reset_controller_state(self):
        self.last_err_x = 0.0
        self.last_err_yaw = 0.0
        self.last_err_fwd = 0.0
        self.smooth_dx = 0.0
        self.smooth_dyaw = 0.0
        self.smooth_dfwd = 0.0
        self.last_cmd = Twist()
        self.control_initialized = False

    def run_pd_control(self, cmd):
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

        def smooth_deadzone(val, threshold):
            return 0.0 if abs(val) < threshold else val

        def deadzone_with_min_speed(val, threshold, minimum):
            if abs(val) < threshold:
                return 0.0
            return float(np.sign(val) * max(abs(val), minimum))

        cmd.angular.z = float(np.clip(smooth_deadzone(v_yaw, 0.06), -self.max_yaw_speed, self.max_yaw_speed))
        cmd.linear.x = float(np.clip(smooth_deadzone(vx, 0.04), -self.max_forward_speed, self.max_forward_speed))
        cmd.linear.y = float(np.clip(smooth_deadzone(vy, 0.04), -self.max_side_speed, self.max_side_speed))
        cmd.linear.z = float(
            np.clip(
                deadzone_with_min_speed(vz, self.vertical_deadband, self.min_vertical_speed),
                -self.max_vertical_speed,
                self.max_vertical_speed,
            )
        )

    def auto_takeoff(self):
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
            f"Takeoff requested; entering follow mode after {self.follow_after_takeoff_sec:.1f}s."
        )

    def stop_motion(self):
        # During shutdown the ROS context can already be invalid; best-effort stop only.
        try:
            if rclpy.ok():
                self.vel_pub.publish(Twist())
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SmartTracker()
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
