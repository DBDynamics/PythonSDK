from DBDynamicsPro import Bee
import time
m = Bee('COM9')  # or COM2 COM3

def prepare():
    for axis in range(2):
        m.setPowerOn(axis)
        m.setAccTime(axis, 200)
        m.setTargetVelocity(axis, 1000)
        m.setTargetPosition(axis, 0)
        m.setInterpolationPositionMode(axis)
    time.sleep(2)
    tp = 51200
    m.setSIPose(100, [51200, 51200, 0, 0, 0, 0,0,0])
    m.setSIPose(100, [0, 0, 0, 0, 0, 0,0,0])
    m.syncInterpolationFlag=1
    m.waitSIP()

prepare()



m.stop()