from Bee import Bee
import time

m = Bee('COM9')  # or COM2 COM3
l = m.scanDevices()
print(l)
m.stop()