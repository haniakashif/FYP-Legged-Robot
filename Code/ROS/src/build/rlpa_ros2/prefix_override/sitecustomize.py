import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/syn/6AFA675EFA6726131/FYP-Legged-Robot/Code/ROS/src/install/rlpa_ros2'
