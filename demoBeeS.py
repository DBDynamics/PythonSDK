from DBDynamics import Bee
m = Bee('COM6')

m.setAccTime(4, 500)
m.setTargetVelocity(4, 1000)
m.setTargetPosition(4, 51200)
m.waitTargetPositionReached(4)
m.setTargetVelocity(4, 200)
m.setHomingMode(4)
m.waitTargetPositionReached(4)

print("home done!")
m.stop()  # or COM2 COM3