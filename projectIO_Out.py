# Program Start Here --------------------------------------------------------------------------------------------------
import usb485

# change to your port number
# for windows user : COMx
# for linux user: /dev/ttyUSBx
# for mac user: not tested yet, welcome support!!

Name = "/dev/ttyUSB0"
u = usb485.DBDStepperGroup(Name)
u.demoTest()
u.demoTest2()
u.stop()
