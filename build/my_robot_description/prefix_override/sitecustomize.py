import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/anessard/Documents/masters 2025/dynamics/ros2_homework/ros2_simple_robot/install/my_robot_description'
