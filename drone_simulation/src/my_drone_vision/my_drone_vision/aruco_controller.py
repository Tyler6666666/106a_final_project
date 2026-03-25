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

        # ==========================================
        # 1. Ultimate PD Control Parameters (with braking/damping)
        # ==========================================
        self.Kp_side = 0.80
        self.Kd_side = 0.35  

        self.Kp_yaw = 2.50
        self.Kd_yaw = 0.60  

        self.Kp_fwd = 0.70  
        self.Kd_fwd = 0.30  

        self.target_dist = 0.8 

        # ==========================================
        # 2. Feed-forward Anticipation Parameters
        # ==========================================
        # The larger this value, the more aggressively the drone anticipates 
        # and strafes to intercept the target based on the marker's yaw. 
        self.K_anticipate = 1.2 

        # Historical data storage (for Derivative calculation)
        self.last_err_x = 0.0
        self.last_err_yaw = 0.0
        self.last_err_fwd = 0.0

        # Low-pass filter parameters (to smooth out camera noise)
        self.filter_alpha = 0.4
        self.smooth_dx = 0.0
        self.smooth_dyaw = 0.0
        self.smooth_dfwd = 0.0

        # ==========================================
        # 3. Dropped-frame Coasting Mechanism State
        # ==========================================
        self.new_pose_received = False
        self.last_cmd = Twist()  

        # ==========================================
        # 4. Search Logic Parameters
        # ==========================================
        self.last_seen = 0.0
        self.is_searching = False
        self.search_start_time = 0.0
        self.search_speed = 0.4      
        self.search_duration = 15.0  
        self.search_cooldown = False 

        self.latest_pose = None
        self.start_time = time.time()
        self.has_sent_takeoff = False

        # Control loop frequency: 20Hz (0.05s)
        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)
        self.create_timer(1.0, self.auto_takeoff)

    def pose_cb(self, msg):
        self.latest_pose = msg
        self.last_seen = time.time()
        self.new_pose_received = True  # Flag that a fresh frame was received
        self.is_searching = False
        self.search_cooldown = False

    def control_loop(self):
        cmd = Twist()
        now = time.time()
        
        # 5-second safety startup grace period
        if now - self.start_time < 5.0:
            return

        time_since_last_seen = now - self.last_seen

        # ==========================================
        # Core Tracking Mode
        # ==========================================
        if self.latest_pose is not None and time_since_last_seen < 0.6:
            if self.new_pose_received:
                # Fresh frame available: calculate high-mobility PD normally
                self.run_pd_control(cmd)
                self.last_cmd = cmd  
                self.new_pose_received = False
            else:
                # No new frame (possible processing delay or dropped frame)
                if time_since_last_seen < 0.15:
                    # Within normal framerate jitter: perfectly maintain previous command
                    cmd.linear.x = self.last_cmd.linear.x
                    cmd.linear.y = self.last_cmd.linear.y
                    cmd.linear.z = self.last_cmd.linear.z
                    cmd.angular.z = self.last_cmd.angular.z
                else:
                    # Stutter exceeds 0.15s: coast with a 95% decay factor 
                    cmd.linear.x = self.last_cmd.linear.x * 0.95
                    cmd.linear.y = self.last_cmd.linear.y * 0.95
                    cmd.linear.z = self.last_cmd.linear.z * 0.95
                    cmd.angular.z = self.last_cmd.angular.z * 0.95
                
            self.is_searching = False

        # ==========================================
        # 360-Degree Slow Search Mode
        # ==========================================
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
                cmd.angular.z = 0.0
                self.is_searching = False
                self.search_cooldown = True 
        
        # ==========================================
        # Idle / Hover Mode
        # ==========================================
        else:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

        self.vel_pub.publish(cmd)

    def run_pd_control(self, cmd):
        """High-mobility PD Controller with Dynamic Strafing & Static Front-Lock Orbiting"""
        err_x = -self.latest_pose.position.x
        err_yaw = -self.latest_pose.orientation.z 
        err_fwd = self.latest_pose.position.z - self.target_dist

        # ==========================================
        # 1. Dynamic Predictive Strafing (Original Magic)
        # ==========================================
        # Combines spatial error with the marker's orientation trend
        active_err_x = err_x + (self.latest_pose.orientation.z * self.K_anticipate)

        # ==========================================
        #  2. Static Front-Lock Orbiting Logic
        # ==========================================
        # Calculate the absolute geometric offset needed to orbit to the front face
        front_orbit_offset = self.target_dist * np.sin(self.latest_pose.orientation.z)
        
        # Dynamic Weight: The slower the lateral movement, the stronger the urge to orbit!
        # If smooth_dx > 0.4, weight becomes 0 (no interference with dynamic 8-figure tracking)
        static_weight = max(0.0, 1.0 - (abs(self.smooth_dx) / 0.4))
        
        # Gently apply the front-lock "pull" to the lateral error based on current speed
        active_err_x += (front_orbit_offset * 0.8 * static_weight)

        # ==========================================
        # 3. Derivative Rate & Filtering
        # ==========================================
        # Calculate error rate of change (D-term for braking)
        raw_dx = (active_err_x - self.last_err_x) / self.dt
        raw_dyaw = (err_yaw - self.last_err_yaw) / self.dt
        raw_dfwd = (err_fwd - self.last_err_fwd) / self.dt

        self.last_err_x = active_err_x  
        self.last_err_yaw = err_yaw
        self.last_err_fwd = err_fwd

        # Velocity smooth filtering (Low-pass filter)
        self.smooth_dx = self.filter_alpha * raw_dx + (1 - self.filter_alpha) * self.smooth_dx
        self.smooth_dyaw = self.filter_alpha * raw_dyaw + (1 - self.filter_alpha) * self.smooth_dyaw
        self.smooth_dfwd = self.filter_alpha * raw_dfwd + (1 - self.filter_alpha) * self.smooth_dfwd

        # ==========================================
        # 4. Command Synthesis
        # ==========================================
        # P(Current gap) + D(Movement trend/braking)
        v_yaw = (err_yaw * self.Kp_yaw) + (self.smooth_dyaw * self.Kd_yaw) + (err_x * 2.2) 
        vy = (active_err_x * self.Kp_side) + (self.smooth_dx * self.Kd_side)
        vx = (err_fwd * self.Kp_fwd) + (self.smooth_dfwd * self.Kd_fwd)
        vz = -self.latest_pose.position.y * 1.2

        # Smooth Deadzone: Ensures absolute stillness when hovering near the target
        def smooth_deadzone(val, threshold):
            if abs(val) < threshold:
                return 0.0
            return val

        # Output commands (clipped to safe physical limits)
        cmd.angular.z = float(np.clip(smooth_deadzone(v_yaw, 0.08), -1.8, 1.8))
        cmd.linear.x = float(np.clip(smooth_deadzone(vx, 0.05), -0.8, 0.8))
        cmd.linear.y = float(np.clip(smooth_deadzone(vy, 0.05), -1.0, 1.0)) 
        cmd.linear.z = float(np.clip(smooth_deadzone(vz, 0.04), -0.4, 0.4))

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
    main()#!/usr/bin/env python3
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

        # ==========================================
        # 1. Ultimate PD Control Parameters (with braking/damping)
        # ==========================================
        self.Kp_side = 0.80
        self.Kd_side = 0.35  

        self.Kp_yaw = 2.50
        self.Kd_yaw = 0.60  

        self.Kp_fwd = 0.70  
        self.Kd_fwd = 0.30  

        self.target_dist = 0.8 

        # ==========================================
        # 2. Feed-forward Anticipation Parameters
        # ==========================================
        # The larger this value, the more aggressively the drone anticipates 
        # and strafes to intercept the target based on the marker's yaw. 
        # Note: If it strafes in the wrong direction, change this to a negative value!
        self.K_anticipate = 1.2 

        # Historical data storage (for Derivative calculation)
        self.last_err_x = 0.0
        self.last_err_yaw = 0.0
        self.last_err_fwd = 0.0

        # Low-pass filter parameters (to smooth out camera noise)
        self.filter_alpha = 0.4
        self.smooth_dx = 0.0
        self.smooth_dyaw = 0.0
        self.smooth_dfwd = 0.0

        # ==========================================
        # 3. Dropped-frame Coasting Mechanism State
        # ==========================================
        self.new_pose_received = False
        self.last_cmd = Twist()  

        # ==========================================
        # 4. Search Logic Parameters
        # ==========================================
        self.last_seen = 0.0
        self.is_searching = False
        self.search_start_time = 0.0
        self.search_speed = 0.4      
        self.search_duration = 15.0  
        self.search_cooldown = False 

        self.latest_pose = None
        self.start_time = time.time()
        self.has_sent_takeoff = False

        # Control loop frequency: 20Hz (0.05s)
        self.dt = 0.05
        self.create_timer(self.dt, self.control_loop)
        self.create_timer(1.0, self.auto_takeoff)

    def pose_cb(self, msg):
        self.latest_pose = msg
        self.last_seen = time.time()
        self.new_pose_received = True  # Flag that a fresh frame was received
        self.is_searching = False
        self.search_cooldown = False

    def control_loop(self):
        cmd = Twist()
        now = time.time()
        
        # 5-second safety startup grace period
        if now - self.start_time < 5.0:
            return

        time_since_last_seen = now - self.last_seen

        # ==========================================
        # Core Tracking Mode
        # ==========================================
        if self.latest_pose is not None and time_since_last_seen < 0.6:
            if self.new_pose_received:
                # Fresh frame available: calculate high-mobility PD normally
                self.run_pd_control(cmd)
                self.last_cmd = cmd  
                self.new_pose_received = False
            else:
                # No new frame (possible processing delay or dropped frame)
                if time_since_last_seen < 0.15:
                    # Within normal framerate jitter: perfectly maintain previous command (stable coasting)
                    cmd.linear.x = self.last_cmd.linear.x
                    cmd.linear.y = self.last_cmd.linear.y
                    cmd.linear.z = self.last_cmd.linear.z
                    cmd.angular.z = self.last_cmd.angular.z
                else:
                    # Stutter exceeds 0.15s: coast with a 95% decay factor 
                    # (prevents the drone from flying blind into a wall at full speed)
                    cmd.linear.x = self.last_cmd.linear.x * 0.95
                    cmd.linear.y = self.last_cmd.linear.y * 0.95
                    cmd.linear.z = self.last_cmd.linear.z * 0.95
                    cmd.angular.z = self.last_cmd.angular.z * 0.95
                
            self.is_searching = False

        # ==========================================
        # 360-Degree Slow Search Mode
        # ==========================================
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
                cmd.angular.z = 0.0
                self.is_searching = False
                self.search_cooldown = True 
        
        # ==========================================
        # Idle / Hover Mode
        # ==========================================
        else:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 0.0

        self.vel_pub.publish(cmd)

    def run_pd_control(self, cmd):
        """High-mobility PD Controller with Feed-Forward Kinematic Anticipation"""
        err_x = -self.latest_pose.position.x
        err_yaw = -self.latest_pose.orientation.z 
        err_fwd = self.latest_pose.position.z - self.target_dist

        # Core Magic: Predictive Strafing (Anticipatory Positioning)
        # Combines spatial error with the marker's orientation trend
        active_err_x = err_x + (self.latest_pose.orientation.z * self.K_anticipate)

        # Calculate error rate of change (D-term for braking)
        raw_dx = (active_err_x - self.last_err_x) / self.dt
        raw_dyaw = (err_yaw - self.last_err_yaw) / self.dt
        raw_dfwd = (err_fwd - self.last_err_fwd) / self.dt

        self.last_err_x = active_err_x  
        self.last_err_yaw = err_yaw
        self.last_err_fwd = err_fwd

        # Velocity smooth filtering (Low-pass filter)
        self.smooth_dx = self.filter_alpha * raw_dx + (1 - self.filter_alpha) * self.smooth_dx
        self.smooth_dyaw = self.filter_alpha * raw_dyaw + (1 - self.filter_alpha) * self.smooth_dyaw
        self.smooth_dfwd = self.filter_alpha * raw_dfwd + (1 - self.filter_alpha) * self.smooth_dfwd

        # Command Synthesis: P(Current gap) + D(Movement trend/braking)
        v_yaw = (err_yaw * self.Kp_yaw) + (self.smooth_dyaw * self.Kd_yaw) + (err_x * 2.2) 
        vy = (active_err_x * self.Kp_side) + (self.smooth_dx * self.Kd_side)
        vx = (err_fwd * self.Kp_fwd) + (self.smooth_dfwd * self.Kd_fwd)
        vz = -self.latest_pose.position.y * 1.2

        # Smooth Deadzone: Ensures absolute stillness when hovering near the target
        def smooth_deadzone(val, threshold):
            if abs(val) < threshold:
                return 0.0
            return val

        # Output commands (clipped to safe physical limits)
        cmd.angular.z = float(np.clip(smooth_deadzone(v_yaw, 0.08), -1.8, 1.8))
        cmd.linear.x = float(np.clip(smooth_deadzone(vx, 0.05), -0.8, 0.8))
        cmd.linear.y = float(np.clip(smooth_deadzone(vy, 0.05), -1.0, 1.0)) 
        cmd.linear.z = float(np.clip(smooth_deadzone(vz, 0.04), -0.4, 0.4))

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
