import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/g/My Drive/Primary/skl-Course/SP26/EECS 206A - Introduction to Robotics/106a_final_project/drone_simulation/install/my_drone_vision'
