from DBDynamics import Ant
import time

m = Ant('/dev/ttyUSB0')  # or COM2 COM3
m.setPowerOff(1)
time.sleep(0.5)
m.stop()
