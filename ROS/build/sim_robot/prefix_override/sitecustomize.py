import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/mubashir/Pictures/FYP-Legged-Robot-main/Code/ROS/install/sim_robot'
