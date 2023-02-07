from DBDynamics import Ant
import time
import random

import datetime

# Linux系统下为/dev/ttyUSB？，如/dev/ttyUSB0,/dev/ttyUSB1
m = Ant('/dev/ttyUSB0')
# Windows系统下为COM？，如COM2 COM3
# m = Ant('COM6')

# 电机编号：范围1-120
mid = 1

# 设置平滑位置模式
m.setPositionMode(id=mid)

# 设置运行电流：参数推荐范围200-1200，电机运行时的电流，可根据负载大小进行调节，电流越大越不容易丢步，同时发热也越大
m.setRunningCurrent(id=mid, value=500)

# 设置保持电流：参数推荐范围200-800，电机静止时的电流，在保证不丢步的前提下尽可能减小电流降低发热量
m.setKeepingCurrent(id=mid, value=200)

# 设置使能，使能后电机由驱动器控制
m.setPowerOn(id=mid)

# 设置运行速度：参数推荐范围1-300,最大速度与供电电压和电机型号有关
# 速度单位： pulse/10ms 近似等于RPM，1.8度步进角的4线2相电机对应50000 pulse/圈
m.setTargetVelocity(id=mid, value=200)

# 设置加速时间：单位ms，参数推荐范围100-2000
# 对于质量较大的负载，可以通过加大加速时间来防止丢步
m.setAccTime(id=mid, value=100)


# 自定义函数：旋转到绝对角度(系统上电或者回零处为0)
def moveAngle(motor_id, theta):
    # 360度对应50000脉冲
    k = 50000 / 360.0
    pos = theta * k
    m.setTargetPosition(id=motor_id, value=int(pos))


# 自定义函数：等待到位
def wait(motor_id):
    time.sleep(0.5)
    m.waitTargetPositionReached(id=motor_id)


for i in range(0, 10):
    moveAngle(mid, 90 * i)
    wait(mid)

m.stop()
