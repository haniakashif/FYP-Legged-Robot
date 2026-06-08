import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/FYP-Legged-Robot/Code/ROS/src/install/rlpa_ros2'
