#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from tello_msgs.srv import TelloAction
import time
import numpy as np

class SmartTracker(Node):
    def __init__(self):
        super().__init__('aruco_controller_node')
        self.vel_pub = self.create_publisher(Twist, '/drone1/cmd_vel', 10)
        self.pose_sub = self.create_subscription(Pose, '/aruco/pose_3d', self.pose_cb, 10)
        self.takeoff_client = self.create_client(TelloAction, '/drone1/tello_action')

        # PD Control Parameters 
        self.Kp_side = 0.60
        self.Kp_yaw = 2.80
        self.Kp_fwd = 0.40
        self.target_dist = 0.8 

        # Search Logic Parameters
        self.last_seen = 0.0
        self.is_searching = False
        self.search_start_time = 0.0
        self.search_speed = 0.4      # Slow rotation speed (rad/s)
        self.search_duration = 15.0  # Time for one full rotation (2*pi / 0.4 ≈ 15.7s)
        self.search_cooldown = False # Prevent immediate re-searching after a full failed rotation

        self.latest_pose = None
        self.start_time = time.time()
        self.has_sent_takeoff = False

        # 20Hz High-frequency control timer
        self.create_timer(0.05, self.control_loop)
        self.create_timer(1.0, self.auto_takeoff)

    def pose_cb(self, msg):
        self.latest_pose = msg
        self.last_seen = time.time()
        # Reset search status whenever the marker is seen
        self.is_searching = False
        self.search_cooldown = False

    def control_loop(self):
        cmd = Twist()
        now = time.time()
        
        # Safety protection: Do nothing for the first 5 seconds after launch
        if now - self.start_time < 5.0:
            return

        #  Core Logic: State Determination 
        time_since_last_seen = now - self.last_seen

        #  Tracking Mode: Target is in view (seen within last 0.6s)
        if self.latest_pose is not None and time_since_last_seen < 0.6:
            self.run_pd_control(cmd)
            self.is_searching = False

        #  Search Mode: Target lost for > 1.0s and search is not on cooldown
        elif time_since_last_seen > 1.0 and not self.search_cooldown:
            if not self.is_searching:
                self.get_logger().warn("Target lost! Starting 360-degree slow search...")
                self.is_searching = True
                self.search_start_time = now
            
            elapsed_search = now - self.search_start_time
            
            if elapsed_search < self.search_duration:
                # Execute slow rotation
                cmd.angular.z = self.search_speed
            else:
                # Full rotation finished without finding target; stop and enter cooldown
                self.get_logger().error("Search complete. Target not found. Entering idle state.")
                cmd.angular.z = 0.0
                self.is_searching = False
                self.search_cooldown = True # Must see marker again to reset search logic
        
        # Idle Mode: Target lost and search cycle finished
        else:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

        self.vel_pub.publish(cmd)

    def run_pd_control(self, cmd):
        """High-performance PD control logic"""
        err_x = -self.latest_pose.position.x
        err_yaw = -self.latest_pose.orientation.z 
        err_fwd = self.latest_pose.position.z - self.target_dist

        # Coupled Yaw Logic: Head towards the marker based on both rotation and lateral offset
        v_yaw = (err_yaw * self.Kp_yaw) + (err_x * 2.0) 
        
        # Base Velocities
        vy = err_x * self.Kp_side
        vx = err_fwd * self.Kp_fwd
        vz = -self.latest_pose.position.y * 1.2 # Altitude correction

        # Deadzone Boost: Ensure the command is strong enough to move the drone
        def boost(val, min_s=0.25):
            if abs(val) > 0.03:
                return val if abs(val) > min_s else (min_s if val > 0 else -min_s)
            return 0.0

        cmd.angular.z = float(np.clip(boost(v_yaw, 0.3), -1.0, 1.0))
        cmd.linear.x = float(np.clip(boost(vx, 0.2), -0.4, 0.4))
        cmd.linear.y = float(np.clip(boost(vy, 0.2), -0.4, 0.4))
        cmd.linear.z = float(np.clip(vz, -0.3, 0.3))

    def auto_takeoff(self):
        if not self.has_sent_takeoff and (time.time() - self.start_time > 4.0):
            if self.takeoff_client.wait_for_service(timeout_sec=1.0):
                req = TelloAction.Request()
                req.cmd = 'takeoff'
                self.takeoff_client.call_async(req)
                self.has_sent_takeoff = True

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

if __name__ == '__main__':
    main()
