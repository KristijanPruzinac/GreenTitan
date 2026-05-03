import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kristijan/Documents/GitHub/GreenTitan/ros2_ws/install/robot_package'
