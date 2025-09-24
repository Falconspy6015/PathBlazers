import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/sachin/Desktop/PathBlazers/raspi/install/navigator'
