import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/marina/Senior-Design/NavQPlus/install/yolo_npu_ros'
