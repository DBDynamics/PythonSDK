from DBDynamics import Ant
import time

m = Ant('/dev/ttyUSB0')  # or COM2 COM3
m.setPowerOff(1)
for i in range(0, 20):
    print(m.getInputIO(1))
    time.sleep(0.5)
m.stop()
