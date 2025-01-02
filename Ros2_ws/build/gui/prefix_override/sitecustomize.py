import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/araby/Cleaning_Robot_Project/Ros2_ws/install/gui'
