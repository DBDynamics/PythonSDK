from DBDynamics import Bee
import time

m = Bee('/dev/ttyS9', baudrate=250000)  # or COM2 COM3
mid = 0
m.setHomingDirection(mid, 1)
m.setHomingLevel(mid, 0)
m.setTargetVelocity(mid, 500)
m.waitTargetPositionReached(mid)
m.setHomingMode(mid)
time.sleep(0.5)
m.waitTargetPositionReached(mid)
print("home done!")
m.stop()
