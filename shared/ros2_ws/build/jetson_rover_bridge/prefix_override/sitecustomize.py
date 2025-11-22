import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jay/Git Sandbox/Jetson Cube Orange Outdoor Rover/shared/ros2_ws/install/jetson_rover_bridge'
