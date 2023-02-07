from DBDynamics import Ant
import time

m = Ant('/dev/ttyUSB0')  # or COM2 COM3
m.setPowerOn(1)
m.setHomingDirection(1, 1)

m.setHomingLevel(1, 0)
m.setTargetVelocity(1, 50)
m.setTargetPosition(1, -50000)
m.waitTargetPositionReached(1)
m.setHomingMode(1)
time.sleep(0.5)
m.waitTargetPositionReached(1)
print("home done!")
m.stop()
