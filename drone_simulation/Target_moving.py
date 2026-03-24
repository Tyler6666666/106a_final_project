#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class FigureEightDriver(Node):
    def __init__(self):
        super().__init__('figure_eight_driver')
        
        # 【注意】这里换成你仿真里目标小车的速度控制话题
        self.cmd_pub = self.create_publisher(Twist, '/target_robot/cmd_vel', 10)
        
        # 8字轨迹参数调节
        self.v_x = 0.4        # 恒定前进速度 (m/s) -> 决定 8 字有多大
        self.w_max = 1.2      # 最大转向速度 (rad/s) -> 决定转弯有多急
        self.period = 8.0     # 画完一个 8 字需要的时间 (秒)
        
        self.start_time = time.time()
        
        # 50Hz 控制频率，保证轨迹圆滑
        self.timer = self.create_timer(0.02, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        t = time.time() - self.start_time
        
        # 角频率计算
        omega = (2 * math.pi) / self.period
        
        # 核心逻辑：线速度恒定，角速度呈正弦震荡
        msg.linear.x = float(self.v_x)
        msg.angular.z = float(self.w_max * math.sin(omega * t))
        
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = FigureEightDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
