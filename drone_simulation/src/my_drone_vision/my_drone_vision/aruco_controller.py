#!/usr/bin/env python3
import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from tello_msgs.msg import FlightData
from tello_msgs.srv import TelloAction


class SmartTracker(Node):
    def __init__(self):
        super().__init__("aruco_controller_node")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("tello_action_service", "/tello_action")
        self.declare_parameter("auto_takeoff", False)
        self.declare_parameter("takeoff_height", 0.5)
        self.declare_parameter("follow_after_takeoff_sec", 1.0)
        self.declare_parameter("target_dist", 0.45)
        self.declare_parameter("land_on_tag_loss", True)
        self.declare_parameter("tag_loss_land_sec", 1.0)
        self.declare_parameter("enable_search", False)
        self.declare_parameter("require_tag_before_takeoff", True)

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        tello_action_service = self.get_parameter("tello_action_service").value
        self.auto_takeoff_enabled = self.get_parameter("auto_takeoff").value
        self.takeoff_height = float(self.get_parameter("takeoff_height").value)
        self.follow_after_takeoff_sec = float(self.get_parameter("follow_after_takeoff_sec").value)
        self.target_dist = float(self.get_parameter("target_dist").value)
        self.land_on_tag_loss = self.get_parameter("land_on_tag_loss").value
        self.tag_loss_land_sec = float(self.get_parameter("tag_loss_land_sec").value)
        self.enable_search = self.get_parameter("enable_search").value
        self.require_tag_before_takeoff = self.get_parameter("require_tag_before_takeoff").value

        self.vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.pose_sub = self.create_subscription(Pose, "/aruco/pose_3d", self.pose_cb, 10)
        self.flight_sub = self.create_subscription(FlightData, "/flight_data", self.flight_cb, 10)
        self.takeoff_client = self.create_client(TelloAction, tello_action_service)

        self.Kp_side = 0.80
        self.Kd_side = 0.35
        self.Kp_yaw = 2.50
        self.Kd_yaw = 0.60
        self.Kp_fwd = 0.70
        self.Kd_fwd = 0.30
        self.K_anticipate = 1.2

        self.last_err_x = 0.0
        self.last_err_yaw = 0.0
        self.last_err_fwd = 0.0
        self.filter_alpha = 0.4
        self.smooth_dx = 0.0
        self.smooth_dyaw = 0.0
        self.smooth_dfwd = 0.0

        self.new_pose_received = False
        self.last_cmd = Twist()
        self.latest_pose = None
        self.latest_flight = None
        self.last_seen = 0.0
        self.is_searching = False
        self.search_start_time = 0.0
        self.search_speed = 0.25
        self.search_duration = 8.0
        self.search_cooldown = False

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
        self.last_seen = time.time()
        self.new_pose_received = True
        self.is_searching = False
        self.search_cooldown = False

    def flight_cb(self, msg):
        self.latest_flight = msg

    def current_height_m(self):
        if self.latest_flight is None:
            return None
        height_cm = max(int(self.latest_flight.h), int(self.latest_flight.tof))
        return height_cm / 100.0

    def control_loop(self):
        cmd = Twist()
        now = time.time()

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

    def run_pd_control(self, cmd):
        err_x = -self.latest_pose.position.x
        err_yaw = -self.latest_pose.orientation.z
        err_fwd = self.latest_pose.position.z - self.target_dist

        active_err_x = err_x + (self.latest_pose.orientation.z * self.K_anticipate)
        front_orbit_offset = self.target_dist * np.sin(self.latest_pose.orientation.z)
        static_weight = max(0.0, 1.0 - (abs(self.smooth_dx) / 0.4))
        active_err_x += front_orbit_offset * 0.8 * static_weight

        raw_dx = (active_err_x - self.last_err_x) / self.dt
        raw_dyaw = (err_yaw - self.last_err_yaw) / self.dt
        raw_dfwd = (err_fwd - self.last_err_fwd) / self.dt

        self.last_err_x = active_err_x
        self.last_err_yaw = err_yaw
        self.last_err_fwd = err_fwd

        self.smooth_dx = self.filter_alpha * raw_dx + (1 - self.filter_alpha) * self.smooth_dx
        self.smooth_dyaw = self.filter_alpha * raw_dyaw + (1 - self.filter_alpha) * self.smooth_dyaw
        self.smooth_dfwd = self.filter_alpha * raw_dfwd + (1 - self.filter_alpha) * self.smooth_dfwd

        v_yaw = (err_yaw * self.Kp_yaw) + (self.smooth_dyaw * self.Kd_yaw) + (err_x * 2.2)
        vy = (active_err_x * self.Kp_side) + (self.smooth_dx * self.Kd_side)
        vx = (err_fwd * self.Kp_fwd) + (self.smooth_dfwd * self.Kd_fwd)
        vz = -self.latest_pose.position.y * 1.2

        def smooth_deadzone(val, threshold):
            return 0.0 if abs(val) < threshold else val

        cmd.angular.z = float(np.clip(smooth_deadzone(v_yaw, 0.08), -0.8, 0.8))
        cmd.linear.x = float(np.clip(smooth_deadzone(vx, 0.05), -0.45, 0.45))
        cmd.linear.y = float(np.clip(smooth_deadzone(vy, 0.05), -0.45, 0.45))
        cmd.linear.z = float(np.clip(smooth_deadzone(vz, 0.04), -0.25, 0.25))

    def auto_takeoff(self):
        if not self.auto_takeoff_enabled or self.has_sent_takeoff:
            return
        if time.time() - self.start_time <= 4.0:
            return
        if self.require_tag_before_takeoff and time.time() - self.last_seen > 0.75:
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


def main(args=None):
    rclpy.init(args=args)
    node = SmartTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
