# Python support for DBD Products
setPowerOn(id) 	设置对应id号的状态为使能,使能后电机开始受驱动器控制.使能后状态指示灯由快速闪烁变为慢速闪烁.

setPowerOff(id) 	设置对应id号的状态为失能,失能后电机不再受驱动器控制.失能后状态指示灯由慢速闪烁变为快速闪烁.

setTargetVelocity(id, value) 	设置目标速度.数值范围通常位1-300,单位pulse/ms近似等于RPM.

setTargetPosition(id, value) 	设置目标位置.Ant控制1.8度步进电机时,50000脉冲当量对应一圈.

setVelocityMode(id) 	设置运行模式为平滑速度模式,详细内容参考运行模式
setPositionMode(id) 	设置运行模式为平滑位置模式,详细内容参考运行模式
setHomingMode(id) 	设置运行模式为回零模式,详细内容参考运行模式
setHomingDirection(id, value) 	设置回零方向.取值为1或者-1.
setHomingLevel(id, value) 	设置回零电平.取值为1或者0.
setRunningCurrent(id, value) 	设置运行电流.取值范围100-1500,单位mA,通常300-800比较合理.
setKeepingCurrent(id, value) 	设置保持电流.取值范围100-1500,单位mA,通常300-800比较合理.
setAccTime(id, value) 	设置加速时间.在位置模式下或者速度模式下的加减速过程的时间,单位ms.通常100-2000比较合理.
setOutputIO(id, value) 	设置IO输出.取值0或者1.
getInputIO(id) 	获取输入IO的状态.返回值为0或者1.
getActualVelocity(id) 	获取当前的实际运行速度.
getActualPosition(id) 	获取当前的实际位置.
getTargetVelocity(id) 	获取目标速度.
getTargetPosition(id) 	获取目标位置.
getRunningCurrent(id) 	获取运行电流.
getKeepingCurrent(id) 	获取保持电流.
getAccTime(id) 	获取加速时间.
getHomingDirection(id) 	获取回零方向.
getHomingLevel(id) 	获取回零电平.
waitHomingDone(id) 	等待回零完成.
waitTargetPositionReached(id) 	等待目标位置到达.
getDeviceID(id) 	获取设备ID.
scanDevices() 	扫描在线设备.
saveParameters(id) 	保存参数.
changeID(id, value) 	修改ID.ID范围1-120.
