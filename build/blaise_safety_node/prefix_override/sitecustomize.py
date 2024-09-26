import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/blaise/lab2_ws/src/blaise_safety_node/install/blaise_safety_node'
