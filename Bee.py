import ctypes
import queue
import struct
import threading
import time
import serial

class Bee:
    # Communication Profiles, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4

    _INDEX_IO_OUT = 23
    _INDEX_IO_OUT_ACC = 30
    _INDEX_RUNNING_CURRENT = 17
    _INDEX_PULSE_DIR = 17
    _INDEX_KEEPING_CURRENT = 18
    _INDEX_HOMING_DIRECTION = 14
    _INDEX_HOMING_LEVEL = 15
    _INDEX_HOMING_OFFSET = 16

    _INDEX_ACC_TIME = 11
    _INDEX_ACTUAL_CURRENT = 6
    _INDEX_TARGET_VELOCITY = 7
    _INDEX_TARGET_POSITION = 9
    _INDEX_ACTUAL_VELOCITY = 8
    _INDEX_ACTUAL_POSITION = 10
    _INDEX_IO_INPUT = 22
    _INDEX_POWER_LIMIT = 28

    _INDEX_KPP = 32
    _INDEX_KPI = 33
    _INDEX_KVF = 34
    _INDEX_KFF = 35
    _INDEX_LIMIT_POSITION_P = 36
    _INDEX_LIMIT_POSITION_N = 37

    # for state machine 2
    _INDEX_TP0 = 25
    _INDEX_TP1 = 26

    _INDEX_ENCODER_VALUE = 21
    _INDEX_ENCODER_POLARITY = 20
    _INDEX_ENCODER_OFFSET = 19
    _INDEX_ENCODER_ERROR = 24

    # for state machine 1
    _INDEX_SM1_TP0 = 25
    _INDEX_SM1_TP1 = 26
    _INDEX_SM1_TV0 = 27
    _INDEX_SM1_TV1 = 28
    _INDEX_SM1_TC = 29
    _INDEX_SM1_TT0 = 30
    _INDEX_SM1_TT1 = 31

    # for StepperRGB
    _INDEX_RED = 24
    _INDEX_GREEN = 25
    _INDEX_BLUE = 26

    # for stepper d
    _INDEX_CURRENT_BASE = 17
    _INDEX_CURRENT_P = 18
    _INDEX_CURRENT_N = 19

    _FUNC_WRITE = 1
    _FUNC_READ = 0
    _FUNC_WRITE_OK = 3
    _FUNC_READ_OK = 2
    _FUNC_OPERATION = 4
    _FUNC_OPERATION_OK = 5
    _FUNC_FREE = 255
    _OPERATION_MODE_PWM = 0
    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_INTERPOLATION_POSITION = 34
    _OPERATION_MODE_HOMING = 40
    _OPERATION_MODE_ESTOP_FAST = 60
    _OPERATION_MODE_ESTOP_PROFILE = 61
    _OPERATION_MODE_ENCODER_CONTROL = 70

    _OPERATION_INDEX_MEMORY = 1
    _STATUS_DEVICE_ENABLE = 0X01
    _STATUS_HOMG_FIND = 0X02
    _STATUS_TARGET_REACHED = 0X04
    _STATUS_IO_INPUT = 0X08
    _STATUS_IO_LIMIT_P = 0X10
    _STATUS_IO_LIMIT_N = 0X20
    _STATUS_ESTOP = 0X40

    _INDEX_PHASE_CORRECT_CURRENT = 21
    _INDEX_HOMING_TRIGGER = 13

    # Variables
    # _connection = 0
    # # Motors consist of 128 motor cells, each motor has 1024 parameters
    # _motors = 0
    # _thread1 = 0
    # _thread_stop_flag = 0
    # _tx_queue = queue.Queue()
    # _tx_lock = threading.Lock()
    # _rx_lock = threading.Lock()

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, self._baudrate, timeout=0.1)

    # Disconnect form the serial port and close the communication
    def _disconnect(self):
        self._thread_stop_flag = 1
        self._connection.close()

    # Analysis the Rx Message and Put the parameters into Each Motor
    def _analysis(self, rx_message):
        if len(rx_message) == 8:
            self._rx_lock.acquire()
            func_code = struct.unpack_from('B', rx_message, 0)
            index = struct.unpack_from('B', rx_message, 1)
            id = struct.unpack_from('B', rx_message, 2)
            subid = struct.unpack_from('B', rx_message, 3)
            data = struct.unpack_from('i', rx_message, 4)

            self.link_status[id[0]] = 0
            if func_code[0] == self._FUNC_READ_OK:
                if id[0] < 64:
                    self.link_status[id[0]] = 1
                if subid[0] > 7:
                    self._error_axis_num = id[0]
                else:
                    self._motors[id[0]][subid[0]][index[0]] = data[0]
            if func_code[0] == self._FUNC_WRITE_OK:
                if id[0] < 64:
                    self.link_status[id[0]] = 1
                if subid[0] > 7:
                    self._error_axis_num = id[0]
                else:
                    self._motors[id[0]][subid[0]][index[0]] = data[0]
            self._rx_lock.release()

    def getLinkStatus(self, id):
        return self.link_status[id]

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        while self._thread_stop_flag == 0:
            # while 1:
            self._tx_lock.acquire()
            # print('msg length:', self._tx_queue.qsize())
            if not self._tx_queue.empty():
                msg = self._tx_queue.get()
                self._tx_lock.release()
                self._connection.write(msg)
                self._analysis(self._connection.read(8))
                self._connection.reset_input_buffer()
            else:
                self._tx_lock.release()
                time.sleep(0.01)

    # Send Message Function, Users would Call this Function to Send Messages
    def _sendMessage(self, func_code, index, id, subid, data):
        message = ctypes.create_string_buffer(8)
        struct.pack_into('B', message, 0, *(func_code,))
        struct.pack_into('B', message, 1, *(index,))
        struct.pack_into('B', message, 2, *(id,))
        struct.pack_into('B', message, 3, *(subid,))
        struct.pack_into('i', message, 4, int(*(data,)))
        self._tx_lock.acquire()
        self._tx_queue.put(message)
        # print('append new message')
        self._tx_lock.release()

    # Stop the communication
    def stop(self):
        time.sleep(1)
        self._thread_stop_flag = 1
        print('Pyhton SDK for DBD Bee Stopped')

    def setPowerOn(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 1)

    def setPowerOnPro(self, id, limit_soft, open_loop, with_break, limit_off):
        value = 0x1
        if limit_soft:
            value |= 0x2
        if open_loop:
            value |= 0x10
        if with_break:
            value |= 0x20
        if limit_off:
            value |= 0x40
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, value)

    def setPowerOff(self, id):
        # Disables power to the motor with the specified ID by sending a control word with value 0
        # Parameters:
        #   id: The ID of the motor to turn off
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)

    def setPowerOffPro(self, id, limit_soft, open_loop, with_break, limit_off):
        value = 0x0
        if limit_soft:
            value |= 0x2
        if open_loop:
            value |= 0x10
        if with_break:
            value |= 0x20
        if limit_off:
            value |= 0x40
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, value)

    def setCurrentBase(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CURRENT_BASE, id, 0, value)

    def setCurrentP(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CURRENT_P, id, 0, value)

    def setCurrentN(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CURRENT_N, id, 0, value)

    def setLEDRed(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_RED, id, 0, value)

    def setLEDGreen(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_GREEN, id, 0, value)

    def setLEDBlue(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_BLUE, id, 0, value)

    def getTP0(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TP0, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_TP0]

    def setTP0(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TP0, id, 0, value)

    def setTP1(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TP1, id, 0, value)

    def getTP1(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TP1, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_TP1]

    def setSM1TP0(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TP0, id, 0, value)

    def setSM1TP1(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TP1, id, 0, value)

    def setSM1TV0(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TV0, id, 0, value)

    def setSM1TV1(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TV1, id, 0, value)

    def setSM1TC(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TC, id, 0, value)

    def setSM1TT0(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TT0, id, 0, value)

    def setSM1TT1(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_SM1_TT1, id, 0, value)

    def setTargetVelocity(self, id, value):
        # Note: Unit: pulse/ms (51200 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 3000 is reasonable, higher speed will lose steps
        self._motors[id][0][self._INDEX_TARGET_VELOCITY] = value
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, id, 0, value)

    def setTargetPosition(self, id, value):
        # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_POSITION, id, 0, value)

    def checkSafety(self, id, timeout):
        flag = 1
        for i in range(0, timeout):
            time.sleep(0.001)
            if self.link_status[id]:
                flag = 0
                break
        if flag:
            print("Timeout")
            return 0
        else:
            return 1

    def setPWMMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PWM)

    def setVelocityMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_VELOCITY)

    def setPositionMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_POSITION)

    def setHomingMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_HOMING)

    def setInterpolationPositionMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0,
                          self._OPERATION_MODE_INTERPOLATION_POSITION)

    def setOpModeEstopProfile(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_ESTOP_PROFILE)

    def setOpModeEstopFast(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_ESTOP_FAST)

    def setOpModeEncoderControl(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_ENCODER_CONTROL)

    def setRunningCurrent(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_RUNNING_CURRENT, id, 0, value)

    def setKeepingCurrent(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KEEPING_CURRENT, id, 0, value)

    def setPulseDir(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_PULSE_DIR, id, 0, value)

    def setHomingDirection(self, id, value):
        if value == 1:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_DIRECTION, id, 0, 1)
        elif value == -1:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_DIRECTION, id, 0, -1)
        else:
            print("wrong value, please try 1 or -1.")

    def setHomingLevel(self, id, value):
        if value == 1:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_LEVEL, id, 0, 1)
        elif value == 0:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_LEVEL, id, 0, 0)
        else:
            print("wrong value, please try 1 or 0.")

    def setHomingOffset(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_OFFSET, id, 0, value)

    def setPowerLimit(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_POWER_LIMIT, id, 0, value)

    def setLimitPositionP(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_LIMIT_POSITION_P, id, 0, value)

    def setLimitPositionN(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_LIMIT_POSITION_N, id, 0, value)

    def setEncoderPolarityN(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, 0, -1)

    def setEncoderPolarityP(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, 0, 1)

    def setPhaseCorrectCurrent(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_PHASE_CORRECT_CURRENT, id, 0, value)

    def setKPP(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPP, id, 0, value)

    def setKFF(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KFF, id, 0, value)

    def setKVF(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KVF, id, 0, value)

    def setAccTime(self, id, value):
        # Note: acc time is a parameter for accelation and deaccelation progress, unit is ms, normally 200ms to 1000ms is reasonable
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ACC_TIME, id, 0, value)

    def _delay(self):
        if self._baudrate == 2000000:
            time.sleep(0.05)
        else:
            time.sleep(0.2)

    def getAccTime(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACC_TIME, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ACC_TIME]

    def getActualCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_CURRENT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ACTUAL_CURRENT]

    def setOutputIO(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_IO_OUT, id, 0, value)

    def getOutputIO(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_IO_OUT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_IO_OUT]

    def setOutputPWMACC(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_IO_OUT_ACC, id, 0, value)

    def getHomingLevel(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_LEVEL, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_HOMING_LEVEL]

    def getHomingDirection(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_DIRECTION, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_HOMING_DIRECTION]

    def getRunningCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_RUNNING_CURRENT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_RUNNING_CURRENT]

    def getKeepingCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KEEPING_CURRENT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_KEEPING_CURRENT]

    def getInputIO(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_IO_INPUT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_IO_INPUT]

    def getActualVelocity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_VELOCITY, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ACTUAL_VELOCITY]

    def getActualPosition(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_POSITION, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ACTUAL_POSITION]

    def getTargetVelocity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_VELOCITY, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_TARGET_VELOCITY]

    def getTargetPosition(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_POSITION, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_TARGET_POSITION]

    def getStatus(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_STATUS_WORD, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_STATUS_WORD]

    def getEncoderValue(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_VALUE, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ENCODER_VALUE]

    def getEncoderError(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_ERROR, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ENCODER_ERROR]

    def setEncoderPolarity(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, 0, value)
        self._delay()
        return self._motors[id][0][self._INDEX_ENCODER_POLARITY]

    def waitHomingDone(self, id):
        condition = 1
        while condition:
            vel = self.getActualVelocity(id)
            if vel == 0:
                condition = 0

    def waitHomingDoneTimeout(self, id, timeout):
        condition = 1
        counter = 0
        while condition:
            counter += 1
            vel = self.getActualVelocity(id)
            if vel == 0:
                condition = 0
                return 1
            if counter > timeout:
                condition = 0
                return -1

    def waitTargetPositionReached(self, id):
        condition = 1
        time.sleep(0.5)
        while condition:
            if (self.getStatus(id) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0
            # if (self.getStatus(id) & self._STATUS_ESTOP) == self._STATUS_ESTOP:
            #     print("EStop Mode, Exit.")
            #     condition = 0

    def waitTargetPositionReachedPro(self, id):
        condition = 1
        while condition:
            print(self.getActualPosition(id))
            if self._motors[id][0][self._INDEX_ACTUAL_POSITION] == self._motors[id][0][self._INDEX_TARGET_POSITION]:
                condition = 0

    # return value: 1 - success, -1 - timeout
    def waitTargetPositionReachedTimeout(self, id, timeout):
        condition = 1
        counter = timeout / 50
        ret = 0
        t = 0
        while condition:
            if (self.getStatus(id) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0
                ret = 1
            t = t + 1
            if t > counter:
                condition = 0
                ret = -1
        return ret

    def getDeviceID(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_DEVICE_ID, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_DEVICE_ID]

    def getDeviceType(self, id):
        self._motors[id][0][self._INDEX_BOARD_TYPE] = 0
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def scanDevices(self):
        online = []
        self.retransmitLimit = 0
        print('Searching Online Devices...')
        for i in range(0, 64):
            self._motors[i][0][self._INDEX_BOARD_TYPE] = 0
        for i in range(0, 64):
            if self.getDeviceType(i) != 0:
                online.append(i)
        print('Online Devices:')
        self.retransmitLimit = 3
        # print(online)
        return online

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_MEMORY, id, 0, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_DEVICE_ID, id, 0, value)
        self._delay()
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName, baudrate=2000000):
        self._connection = 0
        # Motors consist of 128 motor cells, each motor has 1024 parameters
        # self._motors = 0
        # self._thread1 = 0
        self._thread_stop_flag = 0
        self._tx_queue = queue.Queue()
        self._tx_lock = threading.Lock()
        self._rx_lock = threading.Lock()
        self._portName = portName
        self._baudrate = baudrate
        self._error_axis_num = -1
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        array = (((ctypes.c_int32 * 64) * 8) * 64)
        self._motors = array()
        self.link_status = []
        for axis in range(0, 64):
            self.link_status.append(0)
        print('Pyhton SDK for DBD Bee Started')
