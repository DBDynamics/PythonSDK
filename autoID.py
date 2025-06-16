# 这个程序实现自动修改ID的功能
from DBDynamics import Bee
import time

def play_voice(content):
    try:
        import pyttsx3
        engine = pyttsx3.init()
        engine.say(content)
        engine.runAndWait()
    except ImportError:
        print("请安装 pyttsx3 库以使用语音播放功能。")

m = Bee('COM6')

startID = 36
for target in range(startID, 64, 2):
    while True:
        time.sleep(0.1)
        ret = m.getDeviceType(0)
        if ret !=0:
            print("Device 0 is connected")
            play_voice("新设备已连接")
            break
    currentID  = 0
    targetID = target
    print("Target ID: ", targetID)
    m.changeID(currentID, targetID)
    time.sleep(0.1) 
    ret = m.getDeviceType(targetID)
    if ret!=0:
        print("Device ", targetID, " is ready")
        msg = str(targetID) + "号设备修改成功"
        play_voice(msg)

# 播放全部修改成功的语音
msg = "全部设备修改成功"
play_voice(msg)

l = m.scanDevices()
ll = len(l)
print("Total devices: ", ll)
msg = "共扫描到" + str(ll) + "个设备"
play_voice(msg)

m.stop()