# 这个项目是控制64个电机进行运转测试
# 测试的是电机的正反转 

from DBDynamics import Bee
import time

m = Bee('COM9')

for loop in range(100):
    for mid in range(0, 64):
        m.setTargetPosition(mid, 0)
    time.sleep(1)
    m.waitTargetPositionReached(0)

    for mid in range(0, 64, 2):
        m.setTargetPosition(mid, 51200*1000)
        m.setTargetPosition(mid+1, -51200*1000)
        
    time.sleep(1)
    m.waitTargetPositionReached(0)

m.stop()