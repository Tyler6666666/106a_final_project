#!/usr/bin/env python3

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from tello_msgs.srv import TelloAction


def quaternion_to_yaw(x, y, z, w):
    """Extract yaw from quaternion (x, y, z, w)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


class SmartTracker3D(Node):
    def __init__(self):
        super().__init__("aruco_controller_3d_node")
        self.vel_pub = self.create_publisher(Twist, "/drone1/cmd_vel", 10)
        self.pose_sub = self.create_subscription(Pose, "/aruco/pose_3d", self.pose_cb, 10)
        self.takeoff_client = self.create_client(TelloAction, "/drone1/tello_action")

        self.Kp_side = 0.85
        self.Kd_side = 0.35
        self.Kp_yaw = 2.40
        self.Kd_yaw = 0.60
        self.Kp_fwd = 0.75
        self.Kd_fwd = 0.30
        self.Kp_z = 1.20
        self.Kd_z = 0.35
        self.K_anticipate = 1.0
        self.target_dist = 0.8
        self.target_y = 0.0

        self.last_err_x = 0.0
        self.last_err_yaw = 0.0
        self.last_err_fwd = 0.0
        self.last_err_z = 0.0

        self.filter_alpha = 0.4
        self.smooth_dx = 0.0
        self.smooth_dyaw = 0.0
        self.smooth_dfwd = 0.0
        self.smooth_dz = 0.0

        self.new_pose_received = False
        self.last_cmd = Twist()
        self.latest_pose = None
        self.latest_yaw = 0.0

        self.last_seen = 0.0
        self.is_searching = False
        self.search_start_time = 0.0
        self.search_speed = 0.4
        self.search_duration = 15.0
        self.search_cooldown = False

        self.start_time = time.time()
        self.has_sent_takeoff = False
        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)
        self.create_timer(1.0, self.auto_takeoff)

    def pose_cb(self, msg):
        self.latest_pose = msg
        self.latest_yaw = quaternion_to_yaw(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w,
        )
        self.last_seen = time.time()
        self.new_pose_received = True
        self.is_searching = False
        self.search_cooldown = False

    def control_loop(self):
        cmd = Twist()
        now = time.time()
        if now - self.start_time < 5.0:
            return

        time_since_last_seen = now - self.last_seen

        if self.latest_pose is not None and time_since_last_seen < 0.6:
            if self.new_pose_received:
                self.run_pd_control(cmd)
                self.last_cmd = cmd
                self.new_pose_received = False
            else:
                decay = 1.0 if time_since_last_seen < 0.15 else 0.95
                cmd.linear.x = self.last_cmd.linear.x * decay
                cmd.linear.y = self.last_cmd.linear.y * decay
                cmd.linear.z = self.last_cmd.linear.z * decay
                cmd.angular.z = self.last_cmd.angular.z * decay
            self.is_searching = False
        elif time_since_last_seen > 1.0 and not self.search_cooldown:
            if not self.is_searching:
                self.get_logger().warn("Target lost! Starting 360-degree slow search...")
                self.is_searching = True
                self.search_start_time = now

            elapsed_search = now - self.search_start_time
            if elapsed_search < self.search_duration:
                cmd.angular.z = self.search_speed
            else:
                self.get_logger().error("Search complete. Target not found. Entering idle state.")
                self.is_searching = False
                self.search_cooldown = True
        else:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.linear.z = 0.0
            cmd.angular.z = 0.0

        self.vel_pub.publish(cmd)

    def run_pd_control(self, cmd):
        err_x = -self.latest_pose.position.x
        err_yaw = -self.latest_yaw
        err_fwd = self.latest_pose.position.z - self.target_dist
        err_z = self.target_y - self.latest_pose.position.y

        active_err_x = err_x + (self.latest_yaw * self.K_anticipate)
        front_orbit_offset = self.target_dist * np.sin(self.latest_yaw)
        static_weight = max(0.0, 1.0 - (abs(self.smooth_dx) / 0.4))
        active_err_x += (front_orbit_offset * 0.8 * static_weight)

        raw_dx = (active_err_x - self.last_err_x) / self.dt
        raw_dyaw = (err_yaw - self.last_err_yaw) / self.dt
        raw_dfwd = (err_fwd - self.last_err_fwd) / self.dt
        raw_dz = (err_z - self.last_err_z) / self.dt

        self.last_err_x = active_err_x
        self.last_err_yaw = err_yaw
        self.last_err_fwd = err_fwd
        self.last_err_z = err_z

        self.smooth_dx = self.filter_alpha * raw_dx + (1 - self.filter_alpha) * self.smooth_dx
        self.smooth_dyaw = self.filter_alpha * raw_dyaw + (1 - self.filter_alpha) * self.smooth_dyaw
        self.smooth_dfwd = self.filter_alpha * raw_dfwd + (1 - self.filter_alpha) * self.smooth_dfwd
        self.smooth_dz = self.filter_alpha * raw_dz + (1 - self.filter_alpha) * self.smooth_dz

        v_yaw = (err_yaw * self.Kp_yaw) + (self.smooth_dyaw * self.Kd_yaw) + (err_x * 2.0)
        vy = (active_err_x * self.Kp_side) + (self.smooth_dx * self.Kd_side)
        vx = (err_fwd * self.Kp_fwd) + (self.smooth_dfwd * self.Kd_fwd)
        vz = (err_z * self.Kp_z) + (self.smooth_dz * self.Kd_z)

        def smooth_deadzone(val, threshold):
            return 0.0 if abs(val) < threshold else val

        cmd.angular.z = float(np.clip(smooth_deadzone(v_yaw, 0.08), -1.8, 1.8))
        cmd.linear.x = float(np.clip(smooth_deadzone(vx, 0.05), -0.8, 0.8))
        cmd.linear.y = float(np.clip(smooth_deadzone(vy, 0.05), -1.0, 1.0))
        cmd.linear.z = float(np.clip(smooth_deadzone(vz, 0.04), -0.6, 0.6))

    def auto_takeoff(self):
        if not self.has_sent_takeoff and (time.time() - self.start_time > 4.0):
            if self.takeoff_client.wait_for_service(timeout_sec=1.0):
                req = TelloAction.Request()
                req.cmd = "takeoff"
                self.takeoff_client.call_async(req)
                self.has_sent_takeoff = True


def main(args=None):
    rclpy.init(args=args)
    node = SmartTracker3D()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
