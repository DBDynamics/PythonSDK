# Program Start Here --------------------------------------------------------------------------------------------------
import usb485
import os

# change to your port number
# for windows user : COMx
# for linux user: /dev/ttyUSBx
# for mac user: not tested yet, welcome support!!

xAxis = 7
yAxis = 6
zAxis = 5
mid = 1
xValue = 0
yValue = 0
zValue = 0

xT = -76500
yT = -132300
zT = 0

Name = "/dev/ttyUSB0"
u = usb485.DBDStepperGroup(Name)

def init():
    setSpeedZ(2000)
    u.setCurrent(mid, xAxis, 500)
    u.setCurrent(mid, yAxis, 500)
    u.setCurrent(mid, zAxis, 500)

def runPos(x, y, z):
    xValue = x
    yValue = y
    zValue = z
    u.setPos(mid, xAxis, xValue)
    u.setPos(mid, yAxis, yValue)
    u.setPos(mid, zAxis, zValue)
    u.waitPos(mid, xAxis)
    u.waitPos(mid, zAxis)
    u.waitPos(mid, yAxis)


def setSpeedZ(value):
    u.setSpeed(mid, zAxis, value)


def runUp():
    runPos(xT, yT, zT)


def runDown():
    runPos(xT, yT, zT-22500)

def homeInit():
    u.setCurrent(mid, xAxis, 500)
    u.setCurrent(mid, yAxis, 500)
    u.setCurrent(mid, zAxis, 500)

    u.setEnable(mid, zAxis, 1)
    u.setEnable(mid, xAxis, 1)
    u.setEnable(mid, yAxis, 1)

    u.setSpeed(mid, zAxis, 100)
    u.setHoming(mid, zAxis)
    u.waitHomingDone(mid, xAxis)
    u.setSpeed(mid, zAxis, 2000)
    u.setPos(mid, zAxis, zT)
    u.waitPos(mid, zAxis)

    u.setSpeed(mid, yAxis, 100)
    u.setHoming(mid, yAxis)
    u.waitHomingDone(mid, yAxis)
    u.setSpeed(mid, yAxis, 2000)
    u.setPos(mid, yAxis, yT)
    u.waitPos(mid, yAxis)

    u.setSpeed(mid, xAxis, 100)
    u.setHoming(mid, xAxis)
    u.waitHomingDone(mid, xAxis)
    u.setSpeed(mid, xAxis, 2000)
    u.setPos(mid, xAxis, xT)
    u.waitPos(mid, xAxis)

# homeInit()
# init()
runUp()
runDown()
os.system('python3 /home/db/Documents/pystlink/pystlink.py -c STM32F031x6 flash:erase:verify:/home/db/Desktop/stepperProF031-Stepper.bin')
runUp()
u.stop()
