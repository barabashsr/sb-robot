import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/user/robo/sb-robot/microros_ws/install/sensor_msgs_py'
