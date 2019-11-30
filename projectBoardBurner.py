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

Name = "/dev/ttyUSB0"
u = usb485.DBDStepperGroup(Name)

def init():
    setSpeedZ(1500)
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
    runPos(-76500, -132300, 0000)


def runDown():
    runPos(-76500, -132300, -22500)

init()
runUp()
runDown()
os.system('python3 /home/db/Documents/pystlink/pystlink.py -c STM32F031x6 flash:erase:verify:/home/db/Desktop/stepperProF031-Stepper.bin')
runUp()
u.stop()
