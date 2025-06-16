from DBDynamics import Bee
import time

m = Bee('COM6')  # or COM2 COM3
l = m.scanDevices()
print(l)
m.stop()