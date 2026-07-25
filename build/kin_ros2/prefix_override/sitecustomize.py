import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mubashir/Documents/FYP-Legged-Robot-main/Code/install/kin_ros2'
