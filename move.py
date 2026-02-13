from Bee import Bee

import time

m = Bee('COM9')  # or COM2 COM3
m.setAccTime(0, 200)
m.setAccTime(1, 200)
m.setTargetVelocity(0, 1000)
m.setTargetVelocity(1, 1000)
m.setPositionMode(0)
m.setPositionMode(1)
time.sleep(2)
m.setTargetPosition(0, 0)
m.setTargetPosition(1, 0)
time.sleep(2)
m.stop()