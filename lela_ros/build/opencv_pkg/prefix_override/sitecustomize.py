import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sakazu/LELATEAM/lela/lela_ros/install/opencv_pkg'
