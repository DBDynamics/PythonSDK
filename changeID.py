from DBDynamics import Ant
import time

m = Ant('/dev/ttyUSB0')  # or COM2 COM3
currentID = 1
targetID = 120
m.changeID(id=currentID, value=targetID)
m.stop()