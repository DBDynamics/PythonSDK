from DBDynamics import Ant
import time
import random

import datetime
# 这里改成这台电脑的串口号 COM?
m = Ant('/dev/ttyUSB0')# or COM2 COM3

m.setRunningCurrent(1,500)
m.setPowerOn(1)
m.setTargetVelocity(1,110)

def moveAngle(theta):
    k = 50000/360.0
    pos = theta*k
    m.setTargetPosition(1,(int)(pos))

def wait():
    m.waitTargetPositionReached(1)

for i in range(0,360):
    moveAngle(i)
    wait()
    print("current pos:")
    print(m.getActualPosition(1))
    time.sleep(1)

m.stop()

