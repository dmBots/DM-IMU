import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kielas/dm-imu/02.例程/ROS2-humble例程/install/dm_imu'
