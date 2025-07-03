from DBDynamics import Bee
import time

m = Bee('COM9', baudrate=250000)
l = m.scanDevices()
print(l)
m.stop()