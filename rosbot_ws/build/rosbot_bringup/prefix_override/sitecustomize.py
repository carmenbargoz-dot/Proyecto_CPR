import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ale/Proyecto_CPR/rosbot_ws/install/rosbot_bringup'
