import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tyler/ros2_ws/106a_final_project/drone_simulation/install/my_drone_vision'
