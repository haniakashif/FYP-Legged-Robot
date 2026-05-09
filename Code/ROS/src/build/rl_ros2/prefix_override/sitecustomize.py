import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspaces/FYP-Legged-Robot/Code/ROS/src/install/rl_ros2'
