from DBDynamics import Ant
import time

m = Ant('/dev/ttyUSB0')  # or COM2 COM3
m.scanDevices()
m.stop()