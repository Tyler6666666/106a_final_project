import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tyler/ros_workspaces/drone_simulation/install/my_drone_vision'
