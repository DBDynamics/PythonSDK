#!/usr/bin/python3

import ctypes
import queue
import struct
import threading
import time

import serial


# Class for basic communication
# Each Stepper Group consist of 128 motors
class Ant:
    # Communication Profiles, do not change them!
    _INDEX_CONTROL_WORD = 0
    _INDEX_OPERATION_MODE = 1
    _INDEX_IO_OUT = 14
    _INDEX_MEMORY = 30
    _INDEX_DEVICE_ID = 31
    _INDEX_RUNNING_CURRENT = 40
    _INDEX_KEEPING_CURRENT = 42
    _INDEX_HOMING_DIRECTION = 45
    _INDEX_HOMING_LEVEL = 46
    _INDEX_ACC_TIME = 47
    _INDEX_TARGET_VELOCITY = 48
    _INDEX_TARGET_POSITION = 49
    _INDEX_ACTUAL_VELOCITY = 52
    _INDEX_ACTUAL_POSITION = 53
    _INDEX_IO_INPUT = 54
    _INDEX_TX_FEEDBACK = 96
    _INDEX_SINGLE_DIRECTION = 97
    _SUBINDEX_WRITE = 0
    _SUBINDEX_READ = 1
    _SUBINDEX_WRITE_OK = 2
    _SUBINDEX_READ_OK = 3
    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_HOMING = 40
    _FUNC_CODE_TSDO = 0X580
    _FUNC_CODE_FREE = 0X780
    _FUNC_CODE_SYNC = 0X080

    # # Variables
    # _connection = 0
    # # Motors consist of 128 motor cells, each motor has 1024 parameters
    # _motors = numpy.zeros((128, 1024), dtype=numpy.int32)
    # _thread1 = 0
    # _thread_stop_flag = 0
    # _tx_queue = queue.Queue()
    # _tx_lock = threading.Lock()
    # _rx_lock = threading.Lock()

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, 1500000, timeout=0.1)

    # Disconnect form the serial port and close the communication
    def _disconnect(self):
        self._thread_stop_flag = 1
        self._connection.close()

    # Analysis the Rx Message and Put the parameters into Each Motor
    def _analysis(self, rx_message, flag):
        ret = 0
        if len(rx_message) == 11:
            self._rx_lock.acquire()
            id = struct.unpack_from('h', rx_message, 0)
            index = struct.unpack_from('h', rx_message, 2)
            subindex = struct.unpack_from('h', rx_message, 4)
            data = struct.unpack_from('i', rx_message, 6)
            size = struct.unpack_from('B', rx_message, 10)
            self._rx_lock.release()
            func_code = id[0] & 0xff80
            if func_code != self._FUNC_CODE_FREE:
                device_id = id[0] & 0x007f
                if device_id < 128:
                    if index[0] > -1:
                        if index[0] < 1024:
                            if subindex[0] == self._SUBINDEX_READ_OK:
                                self._motors[device_id][index[0]] = data[0]
                                ret = 1
                            elif subindex[0] == self._SUBINDEX_WRITE_OK:
                                ret = 1
                #             else:
                #                 print("subindex: ")
                #                 print(subindex[0])
                #         else:
                #             print("index: ")
                #             print(index[0])
                #     else:
                #         print("index: ")
                #         print(index[0])
                # else:
                #     print("id: ")
                #     print(device_id)
            else:
                if flag == 1:
                    ret = 1
                # else:
                    # print("free: ")
                    # print(flag)
        else:
            # print("rx_message length: ")
            print(len(rx_message))
        return ret

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        freeMsgFlag = 0
        while self._thread_stop_flag == 0:
            self._tx_lock.acquire()
            # print('msg length:', self._tx_queue.qsize())
            if self._retransmitCounter == 0:
                if not self._tx_queue.empty():
                    self._msg = self._tx_queue.get()
                    freeMsgFlag = 0
                else:
                    struct.pack_into('h', self._msg, 0, self._FUNC_CODE_FREE)
                    struct.pack_into('h', self._msg, 2, *(0,))
                    struct.pack_into('h', self._msg, 4, *(0,))
                    struct.pack_into('i', self._msg, 6, *(0,))
                    freeMsgFlag = 1
            self._tx_lock.release()
            # time.sleep(0.01)

            self._connection.write(self._msg)
            if self._analysis(self._connection.read(11), freeMsgFlag) == 0:
                self._retransmitCounter = 0
                # self._retransmitCounter += 1
                # print("msg retransmit")
                # print(self._retransmitCounter)
                # if self._retransmitCounter > self._retransmitLimit:
                #     self._retransmitCounter = 0
                #     print("retransmit 10 times")
            else:
                self._retransmitCounter = 0

            self._connection.reset_input_buffer()

            #

    # Send Message Function, Users would Call this Function to Send Messages
    def _sendMessage(self, func_code, device_id, index, sub_index, data):
        data = int(data)
        id = (func_code + device_id,)
        message = ctypes.create_string_buffer(10)
        struct.pack_into('h', message, 0, *id)
        struct.pack_into('h', message, 2, *(index,))
        struct.pack_into('h', message, 4, *(sub_index,))
        struct.pack_into('i', message, 6, *(data,))
        self._tx_lock.acquire()
        self._tx_queue.put(message)
        self._tx_lock.release()

    # Stop the communication
    def stop(self):
        time.sleep(0.5)
        self._thread_stop_flag = 1
        print('Python SDK for DBD Ant Stopped.')

    def setSingleDirectionPositive(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_SINGLE_DIRECTION, self._SUBINDEX_WRITE, 1)

    def setSingleDirectionNegative(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_SINGLE_DIRECTION, self._SUBINDEX_WRITE, -1)

    def setSingleDirectionOff(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_SINGLE_DIRECTION, self._SUBINDEX_WRITE, 0)

    def setTxFeedBackOn(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TX_FEEDBACK, self._SUBINDEX_WRITE, 1)

    def setTxFeedBackOff(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TX_FEEDBACK, self._SUBINDEX_WRITE, 0)

    def setPowerOn(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_CONTROL_WORD, self._SUBINDEX_WRITE, 1)

    def setPowerOff(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_CONTROL_WORD, self._SUBINDEX_WRITE, 0)

    def setTargetVelocity(self, id, value):
        # Note: Unit: pulse/ms (50000 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 300 is reasonable, higher speed will lose steps
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_VELOCITY, self._SUBINDEX_WRITE, value)

    def setTargetPosition(self, id, value):
        # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_POSITION, self._SUBINDEX_WRITE, value)

    def setVelocityMode(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
                          self._OPERATION_MODE_PROFILE_VELOCITY)

    def setPositionMode(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
                          self._OPERATION_MODE_PROFILE_POSITION)

    def setHomingMode(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
                          self._OPERATION_MODE_HOMING)

    def setHomingDirection(self, id, value):
        if value == 1:
            self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_DIRECTION, self._SUBINDEX_WRITE,
                              1)
        elif value == -1:
            self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_DIRECTION, self._SUBINDEX_WRITE,
                              -1)
        else:
            print("wrong value, please try 1 or -1.")

    def setHomingLevel(self, id, value):
        if value == 1:
            self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_LEVEL, self._SUBINDEX_WRITE,
                              1)
        elif value == 0:
            self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_LEVEL, self._SUBINDEX_WRITE,
                              0)
        else:
            print("wrong value, please try 1 or 0.")

    def setRunningCurrent(self, id, value):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_RUNNING_CURRENT, self._SUBINDEX_WRITE, value)

    def setKeepingCurrent(self, id, value):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_KEEPING_CURRENT, self._SUBINDEX_WRITE, value)

    def setAccTime(self, id, value):
        # Note: acc time is a parameter for accelation and deaccelation progress, unit is ms, normally 200ms to 1000ms is reasonable
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_ACC_TIME, self._SUBINDEX_WRITE, value)

    def setOutputIO(self, id, value):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_IO_OUT, self._SUBINDEX_WRITE, value)

    def getInputIO(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_IO_INPUT, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_IO_INPUT]

    def getActualVelocity(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_ACTUAL_VELOCITY, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_ACTUAL_VELOCITY]

    def getActualPosition(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_ACTUAL_POSITION, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_ACTUAL_POSITION]

    def getTargetVelocity(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_VELOCITY, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_TARGET_VELOCITY]

    def getTargetPosition(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_POSITION, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_TARGET_POSITION]

    def getRunningCurrent(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_RUNNING_CURRENT, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_RUNNING_CURRENT]

    def getKeepingCurrent(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_KEEPING_CURRENT, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_KEEPING_CURRENT]

    def getAccTime(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_ACC_TIME, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_ACC_TIME]

    def getHomingDirection(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_DIRECTION, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_HOMING_DIRECTION]

    def getHomingLevel(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_LEVEL, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_HOMING_LEVEL]

    def waitHomingDone(self, id):
        condition = 1
        while condition:
            vel = self.getActualVelocity(id)
            if (vel == 0):
                condition = 0

    def waitTargetPositionReached(self, id):
        condition = 1
        while condition:
            vel = self.getActualVelocity(id)
            if (vel == 0):
                condition = 0

    # return value: 1 - success, -1 - timeout
    def waitTargetPositionReachedTimeout(self, id, timeout):
        condition = 1
        counter = timeout / 50
        ret = 0
        t = 0
        while condition:
            vel = self.getActualVelocity(id)
            if vel == 0:
                condition = 0
                ret = 1
            t = t + 1
            if t > counter:
                condition = 0
                ret = -1
        return ret

    def getDeviceID(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_DEVICE_ID, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_DEVICE_ID]

    def scanDevices(self):
        online = []
        self._retransmitLimit = 1
        print('Searching Online Devices...')
        for i in range(0, 121):
            self._motors[i][self._INDEX_DEVICE_ID] = 0
        for i in range(1, 121):
            if i == self.getDeviceID(i):
                online.append(i)
        print('Online Devices:')
        self._retransmitLimit = 3
        print(online)

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_MEMORY, self._SUBINDEX_WRITE, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_DEVICE_ID, self._SUBINDEX_WRITE, value)
        time.sleep(0.05)
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName):
        # Variables
        self._connection = 0
        # Motors consist of 128 motor cells, each motor has 1024 parameters
        # self._motors = numpy.zeros((128, 1024), dtype=numpy.int32)
        array = ((ctypes.c_int32 * 128) * 1024)
        self._motors = array()
        self._thread_stop_flag = 0
        self._retransmitCounter = 0
        self._retransmitLimit = 10
        self._tx_queue = queue.Queue()
        self._tx_lock = threading.Lock()
        self._rx_lock = threading.Lock()
        self._portName = portName
        self._msg = ctypes.create_string_buffer(10)
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        print('Pyhton SDK for DBD Ant Started')


class Bee:
    # Communication Profiles, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4
    _INDEX_IO_OUT = 23
    _INDEX_RUNNING_CURRENT = 17
    _INDEX_KEEPING_CURRENT = 18
    _INDEX_HOMING_DIRECTION = 14
    _INDEX_HOMING_LEVEL = 15
    _INDEX_ACC_TIME = 11
    _INDEX_TARGET_VELOCITY = 7
    _INDEX_TARGET_POSITION = 9
    _INDEX_ACTUAL_VELOCITY = 8
    _INDEX_ACTUAL_POSITION = 10
    _INDEX_IO_INPUT = 22
    # for state machine 2
    _INDEX_TP0 = 25
    _INDEX_TP1 = 26

    _FUNC_WRITE = 1
    _FUNC_READ = 0
    _FUNC_WRITE_OK = 3
    _FUNC_READ_OK = 2
    _FUNC_OPERATION = 4
    _FUNC_OPERATION_OK = 5
    _FUNC_FREE = 255
    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_HOMING = 40
    _OPERATION_INDEX_MEMORY = 1
    _STATUS_DEVICE_ENABLE = 0X01
    _STATUS_HOMG_FIND = 0X02
    _STATUS_TARGET_REACHED = 0X04
    _STATUS_IO_INPUT = 0X08
    # Variables
    _connection = 0
    # Motors consist of 128 motor cells, each motor has 1024 parameters
    _motors = 0
    _thread1 = 0
    _thread_stop_flag = 0
    _tx_queue = queue.Queue()
    _tx_lock = threading.Lock()
    _rx_lock = threading.Lock()

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

            if func_code[0] == self._FUNC_READ_OK:
                self._motors[id[0]][subid[0]][index[0]] = data[0]

            self._rx_lock.release()

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

    def setPowerOff(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)

    def setTP0(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TP0, id, 0, value)

    def setTP1(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TP1, id, 0, value)

    def setTargetVelocity(self, id, value):
        # Note: Unit: pulse/ms (51200 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 3000 is reasonable, higher speed will lose steps
        self._motors[id][0][self._INDEX_TARGET_VELOCITY] = value
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, id, 0, value)

    def setTargetPosition(self, id, value):
        # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
        self._motors[id][0][self._INDEX_TARGET_POSITION] = value
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_POSITION, id, 0, value)

    def setVelocityMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_VELOCITY)

    def setPositionMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_POSITION)

    def setHomingMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_HOMING)

    def setRunningCurrent(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_RUNNING_CURRENT, id, 0, value)

    def setKeepingCurrent(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KEEPING_CURRENT, id, 0, value)

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

    def setAccTime(self, id, value):
        # Note: acc time is a parameter for accelation and deaccelation progress, unit is ms, normally 200ms to 1000ms is reasonable
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ACC_TIME, id, 0, value)

    def _delay(self):
        if self._baudrate == 2000000:
            time.sleep(0.05)
        else:
            time.sleep(0.1)
    def getAccTime(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACC_TIME, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ACC_TIME]

    def setOutputIO(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_IO_OUT, id, 0, value)

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

    def waitHomingDone(self, id):
        condition = 1
        while condition:
            vel = self.getActualVelocity(id)
            if vel == 0:
                condition = 0

    def waitTargetPositionReached(self, id):
        condition = 1
        while condition:
            if (self.getStatus(id) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0

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
        self._retransmitLimit = 1
        print('Searching Online Devices...')
        for i in range(0, 32):
            self._motors[i][0][self._INDEX_BOARD_TYPE] = 0
        for i in range(0, 32):
            if self.getDeviceType(i) != 0:
                online.append(i)
        print('Online Devices:')
        self._retransmitLimit = 3
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
        self._portName = portName
        self._baudrate = baudrate
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        array = (((ctypes.c_int32 * 32) * 8) * 32)
        self._motors = array()
        print('Pyhton SDK for DBD Bee Started')


class Elephant:
    # Communication Profiles, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_IO_OUT = 23
    _INDEX_RUNNING_CURRENT = 17
    _INDEX_KEEPING_CURRENT = 18
    _INDEX_HOMING_DIRECTION = 14
    _INDEX_HOMING_LEVEL = 15
    _INDEX_ACC_TIME = 11
    _INDEX_TARGET_VELOCITY = 7
    _INDEX_TARGET_POSITION = 9
    _INDEX_ACTUAL_VELOCITY = 8
    _INDEX_ACTUAL_POSITION = 10
    _INDEX_CURRENT_BOOST = 19
    _INDEX_RUNTOKEEP_TIME = 20
    _INDEX_BOOST_TIME = 21
    _INDEX_IO_INPUT = 22
    _INDEX_BAUDRATE = 27

    _FUNC_WRITE = 1
    _FUNC_READ = 0
    _FUNC_WRITE_OK = 3
    _FUNC_READ_OK = 2
    _FUNC_OPERATION = 4
    _FUNC_OPERATION_OK = 5
    _FUNC_FREE = 255
    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_INTERPOLATION_POSITION = 34
    _OPERATION_MODE_HOMING = 40
    _OPERATION_INDEX_MEMORY = 1

    # Variables
    _connection = 0
    # Motors consist of 128 motor cells, each motor has 1024 parameters
    # _motors = numpy.zeros((32, 8, 32), dtype=numpy.int32)
    array = (((ctypes.c_int32 * 32) * 8) * 32)
    _motors = array()
    _thread1 = 0

    _thread_stop_flag = 0
    _tx_queue = queue.Queue()
    _tx_lock = threading.Lock()
    _rx_lock = threading.Lock()

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, 2000000, timeout=0.1)

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

            if func_code[0] == self._FUNC_READ_OK:
                self._motors[id[0]][subid[0]][index[0]] = data[0]

            self._rx_lock.release()

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
                msg = ctypes.create_string_buffer(8)
                struct.pack_into('B', msg, 0, self._FUNC_FREE)
                struct.pack_into('B', msg, 1, *(0,))
                struct.pack_into('B', msg, 2, *(0,))
                struct.pack_into('B', msg, 3, *(0,))
                struct.pack_into('i', msg, 4, int(*(0,)))
                self._tx_lock.release()
                time.sleep(0.02)

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
        print('Pyhton SDK for DBD Elephant Stopped')

    def setPowerOn(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, subid, 1)

    def setPowerOff(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, subid, 0)

    def setTargetVelocity(self, id, subid, value):
        # Note: Unit: pulse/ms (51200 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 3000 is reasonable, higher speed will lose steps
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, id, subid, value)

    def setTargetPosition(self, id, subid, value):
        # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_POSITION, id, subid, value)

    def setVelocityMode(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, subid,
                          self._OPERATION_MODE_PROFILE_VELOCITY)

    def setPositionMode(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, subid,
                          self._OPERATION_MODE_PROFILE_POSITION)

    def setInterpolationPositionMode(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, subid,
                          self._OPERATION_MODE_INTERPOLATION_POSITION)

    def setHomingMode(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, subid, self._OPERATION_MODE_HOMING)

    def setHomingDirection(self, id, subid, value):
        if value == 1:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_DIRECTION, id, subid, 1)
        elif value == -1:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_DIRECTION, id, subid, -1)
        else:
            print("wrong value, please try 1 or -1.")

    def setHomingLevel(self, id, subid, value):
        if value == 1:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_LEVEL, id, subid, 1)
        elif value == 0:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_LEVEL, id, subid, 0)
        else:
            print("wrong value, please try 1 or 0.")

    def setRunningCurrent(self, id, subid, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_RUNNING_CURRENT, id, subid, value)

    def setKeepingCurrent(self, id, subid, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KEEPING_CURRENT, id, subid, value)

    def setBaudrate250Kbps(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_BAUDRATE, id, subid, 5)

    def setAccTime(self, id, subid, value):
        # Note: acc time is a parameter for accelation and deaccelation progress, unit is ms, normally 200ms to 1000ms is reasonable
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ACC_TIME, id, subid, value)

    def getAccTime(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACC_TIME, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_ACC_TIME]

    def setBoostTime(self, id, subid, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_BOOST_TIME, id, subid, value)

    def setCurrentBoost(self, id, subid, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CURRENT_BOOST, id, subid, value)

    def setRuntoKeepTime(self, id, subid, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_RUNTOKEEP_TIME, id, subid, value)

    def getHomingDirection(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_DIRECTION, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_HOMING_DIRECTION]

    def getHomingLevel(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_LEVEL, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_HOMING_LEVEL]

    def getInputIO(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_IO_INPUT, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_IO_INPUT]

    def getActualVelocity(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_VELOCITY, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_ACTUAL_VELOCITY]

    def getActualPosition(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_POSITION, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_ACTUAL_POSITION]

    def getTargetVelocity(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_VELOCITY, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_TARGET_VELOCITY]

    def getTargetPosition(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_POSITION, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_TARGET_POSITION]

    def getRunningCurrent(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_RUNNING_CURRENT, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_RUNNING_CURRENT]

    def getKeepingCurrent(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_KEEPING_CURRENT, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_KEEPING_CURRENT]

    def waitHomingDone(self, id, subid):
        condition = 1
        while condition:
            vel = self.getActualVelocity(id, subid)
            if (vel == 0):
                condition = 0

    def waitTargetPositionReached(self, id, subid):
        condition = 1
        while condition:
            vel = self.getActualVelocity(id, subid)
            if (vel == 0):
                condition = 0

    def getDeviceType(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def scanDevices(self):
        online = []
        print('Searching Online Devices...')
        for i in range(0, 32):
            self._motors[i][0][self._INDEX_BOARD_TYPE] = 0
        for i in range(0, 32):
            if self.getDeviceType(i) != 0:
                online.append(i)
        print('Online Devices:')
        print(online)

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_MEMORY, id, 0, 1)

    def changeID(self, id, subid, value):
        self._sendMessage(self._FUNC_READ, self._INDEX_DEVICE_ID, id, subid, value)
        time.sleep(0.05)
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName):
        self._portName = portName
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.getName()
        self._thread1.start()
        print('Pyhton SDK for DBD Elephant Started')


class BeeDCS:
    # Communication Profiles, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4
    _INDEX_TARGET_CURRENT = 5
    _INDEX_IO_OUT = 23
    _INDEX_KPP = 17
    _INDEX_KPI = 18
    _INDEX_KPD = 19
    _INDEX_HOMING_DIRECTION = 14
    _INDEX_HOMING_LEVEL = 15
    _INDEX_ACC_TIME = 11
    _INDEX_TARGET_VELOCITY = 7
    _INDEX_TARGET_POSITION = 9
    _INDEX_ACTUAL_VELOCITY = 8
    _INDEX_ACTUAL_POSITION = 10
    _INDEX_IO_INPUT = 22
    _INDEX_ENCODER_POLARITY = 25
    _INDEX_ENCODER_VALUE = 26
    _FUNC_WRITE = 1
    _FUNC_READ = 0
    _FUNC_WRITE_OK = 3
    _FUNC_READ_OK = 2
    _FUNC_OPERATION = 4
    _FUNC_OPERATION_OK = 5
    _FUNC_FREE = 255
    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_HOMING = 40
    _OPERATION_INDEX_MEMORY = 1
    _STATUS_DEVICE_ENABLE = 0X01
    _STATUS_HOMG_FIND = 0X02
    _STATUS_TARGET_REACHED = 0X04
    _STATUS_IO_INPUT = 0X08
    _BOARD_TYPE_STEPPER_ANT = 0X10
    _BOARD_TYPE_STEPPER_BEE = 0X11
    _BOARD_TYPE_STEPPER_ELEPHANT = 0X12
    _BOARD_TYPE_BDCS_BEE = 0X13
    _BOARD_TYPE_BDC_BEE = 0X14
    _BOARD_TYPE_BLDCS_BEE = 0X15

    # Variables
    _connection = 0
    # Motors consist of 128 motor cells, each motor has 1024 parameters
    # _motors = numpy.zeros((32, 8, 32), dtype=numpy.int32)
    array = (((ctypes.c_int32 * 32) * 8) * 32)
    _motors = array()
    _thread1 = 0
    _thread_stop_flag = 0
    _tx_queue = queue.Queue()
    _tx_lock = threading.Lock()
    _rx_lock = threading.Lock()

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, 2000000, timeout=0.1)

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

            if func_code[0] == self._FUNC_READ_OK:
                self._motors[id[0]][subid[0]][index[0]] = data[0]

            self._rx_lock.release()

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
        print('Pyhton SDK for DBD Brushed DC Servo Motor Stopped')

    def setPowerOn(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 1)

    def setPowerOff(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)

    def setTargetVelocity(self, id, value):
        # Note: Unit: pulse/ms (51200 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 3000 is reasonable, higher speed will lose steps
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, id, 0, value)

    def setTargetPosition(self, id, value):
        # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_POSITION, id, 0, value)

    def setVelocityMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_VELOCITY)

    def setPositionMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_POSITION)

    def setHomingMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_HOMING)

    def setKPP(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPP, id, 0, value)

    def setKPI(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPI, id, 0, value)

    def setKPD(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPD, id, 0, value)

    def setTargetCurrent(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_CURRENT, id, 0, value)

    def setBoardTypeBLDCS(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_BOARD_TYPE, id, 0, self._BOARD_TYPE_BLDCS_BEE)

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

    def setAccTime(self, id, value):
        # Note: acc time is a parameter for accelation and deaccelation progress, unit is ms, normally 200ms to 1000ms is reasonable
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ACC_TIME, id, 0, value)

    def getBoardType(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        time.sleep(0.05)
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BLDCS_BEE):
            print("Brushless DC Servo Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BDC_BEE):
            print("Brushed DC Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BDCS_BEE):
            print("Brushed DC Servo Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_STEPPER_BEE):
            print("Stepper Bee")
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def getAccTime(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACC_TIME, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ACC_TIME]

    def setOutputIO(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_IO_OUT, id, 0, value)

    def getHomingLevel(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_LEVEL, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_HOMING_LEVEL]

    def getHomingDirection(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_DIRECTION, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_HOMING_DIRECTION]

    def getKPP(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPP, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_KPP]

    def getKPI(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPI, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_KPI]

    def getKPD(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPD, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_KPD]

    def getTargetCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_CURRENT, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_TARGET_CURRENT]

    def getInputIO(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_IO_INPUT, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_IO_INPUT]

    def getActualVelocity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_VELOCITY, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ACTUAL_VELOCITY]

    def getActualPosition(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_POSITION, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ACTUAL_POSITION]

    def getTargetVelocity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_VELOCITY, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_TARGET_VELOCITY]

    def getTargetPosition(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_POSITION, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_TARGET_POSITION]

    def getStatus(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_STATUS_WORD, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_STATUS_WORD]

    def setEncoderPolarityP(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, 0, 1)

    def setEncoderPolarityN(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, 0, -1)

    def getEncoderPolarity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_POLARITY, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ENCODER_POLARITY]

    def waitHomingDone(self, id):
        condition = 1
        while condition:
            if (self.getStatus(id) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0

    def waitTargetPositionReached(self, id):
        condition = 1
        while condition:
            if (self.getStatus(id) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0

    def getDeviceID(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_DEVICE_ID, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_DEVICE_ID]

    def getDeviceType(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def scanDevices(self):
        online = []
        print('Searching Online Devices...')
        for i in range(0, 32):
            self._motors[i][0][self._INDEX_BOARD_TYPE] = 0
        for i in range(0, 32):
            if self.getDeviceType(i) != 0:
                online.append(i)
        print('Online Devices:')
        print(online)

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_MEMORY, id, 0, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_DEVICE_ID, id, 0, value)
        time.sleep(0.05)
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName):
        self._portName = portName
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        print('Pyhton SDK for DBD Bee Brushed DC Servo Motor Started')


class BDCS_D:
    # Communication Profiles, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4
    _INDEX_TARGET_CURRENT = 5
    _INDEX_IO_OUT = 23
    _INDEX_KPP = 17
    _INDEX_KPI = 18
    _INDEX_KPD = 19
    _INDEX_HOMING_DIRECTION = 14
    _INDEX_HOMING_LEVEL = 15
    _INDEX_ACC_TIME = 11
    _INDEX_TARGET_VELOCITY = 7
    _INDEX_TARGET_POSITION = 9
    _INDEX_ACTUAL_VELOCITY = 8
    _INDEX_ACTUAL_POSITION = 10
    _INDEX_IO_INPUT = 22
    _INDEX_ENCODER_POLARITY = 25
    _INDEX_ENCODER_VALUE = 26
    _FUNC_WRITE = 1
    _FUNC_READ = 0
    _FUNC_WRITE_OK = 3
    _FUNC_READ_OK = 2
    _FUNC_OPERATION = 4
    _FUNC_OPERATION_OK = 5
    _FUNC_FREE = 255
    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_HOMING = 40
    _OPERATION_INDEX_MEMORY = 1
    _STATUS_DEVICE_ENABLE = 0X01
    _STATUS_HOMG_FIND = 0X02
    _STATUS_TARGET_REACHED = 0X04
    _STATUS_IO_INPUT = 0X08
    _BOARD_TYPE_STEPPER_ANT = 0X10
    _BOARD_TYPE_STEPPER_BEE = 0X11
    _BOARD_TYPE_STEPPER_ELEPHANT = 0X12
    _BOARD_TYPE_BDCS_BEE = 0X13
    _BOARD_TYPE_BDC_BEE = 0X14
    _BOARD_TYPE_BLDCS_BEE = 0X15

    # Variables
    _connection = 0
    # Motors consist of 128 motor cells, each motor has 1024 parameters
    # _motors = numpy.zeros((32, 8, 32), dtype=numpy.int32)
    array = (((ctypes.c_int32 * 32) * 8) * 32)
    _motors = array()
    _thread1 = 0
    _thread_stop_flag = 0
    _tx_queue = queue.Queue()
    _tx_lock = threading.Lock()
    _rx_lock = threading.Lock()

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, 2000000, timeout=0.1)

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

            if func_code[0] == self._FUNC_READ_OK:
                self._motors[id[0]][subid[0]][index[0]] = data[0]

            self._rx_lock.release()

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
        print('Pyhton SDK for DBD Ant Double Brushed DC Servo Motor Stopped')

    def setPowerOn(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, subid, 1)

    def setPowerOff(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, subid, 0)

    def setTargetVelocity(self, id, subid, value):
        # Note: Unit: pulse/ms (51200 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 3000 is reasonable, higher speed will lose steps
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, id, subid, value)

    def setTargetPosition(self, id, subid, value):
        # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_POSITION, id, subid, value)

    def setVelocityMode(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, subid,
                          self._OPERATION_MODE_PROFILE_VELOCITY)

    def setPositionMode(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, subid,
                          self._OPERATION_MODE_PROFILE_POSITION)

    def setHomingMode(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, subid, self._OPERATION_MODE_HOMING)

    def setKPP(self, id, subid, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPP, id, subid, value)

    def setKPI(self, id, subid, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPI, id, subid, value)

    def setKPD(self, id, subid, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPD, id, subid, value)

    def setTargetCurrent(self, id, subid, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_CURRENT, id, subid, value)

    def setBoardTypeBLDCS(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_BOARD_TYPE, id, subid, self._BOARD_TYPE_BLDCS_BEE)

    def setHomingDirection(self, id, subid, value):
        if value == 1:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_DIRECTION, id, subid, 1)
        elif value == -1:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_DIRECTION, id, subid, -1)
        else:
            print("wrong value, please try 1 or -1.")

    def setHomingLevel(self, id, subid, value):
        if value == 1:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_LEVEL, id, subid, 1)
        elif value == 0:
            self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_LEVEL, id, subid, 0)
        else:
            print("wrong value, please try 1 or 0.")

    def setAccTime(self, id, subid, value):
        # Note: acc time is a parameter for accelation and deaccelation progress, unit is ms, normally 200ms to 1000ms is reasonable
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ACC_TIME, id, subid, value)

    def getBoardType(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, subid, 0)
        time.sleep(0.05)
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BLDCS_BEE):
            print("Brushless DC Servo Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BDC_BEE):
            print("Brushed DC Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BDCS_BEE):
            print("Brushed DC Servo Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_STEPPER_BEE):
            print("Stepper Bee")
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def getAccTime(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACC_TIME, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_ACC_TIME]

    def setOutputIO(self, id, subid, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_IO_OUT, id, subid, value)

    def getHomingLevel(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_LEVEL, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_HOMING_LEVEL]

    def getHomingDirection(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_DIRECTION, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_HOMING_DIRECTION]

    def getKPP(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPP, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_KPP]

    def getKPI(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPI, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_KPI]

    def getKPD(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPD, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_KPD]

    def getTargetCurrent(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_CURRENT, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_TARGET_CURRENT]

    def getInputIO(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_IO_INPUT, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_IO_INPUT]

    def getActualVelocity(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_VELOCITY, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_ACTUAL_VELOCITY]

    def getActualPosition(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_POSITION, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_ACTUAL_POSITION]

    def getTargetVelocity(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_VELOCITY, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_TARGET_VELOCITY]

    def getTargetPosition(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_POSITION, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_TARGET_POSITION]

    def getStatus(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_STATUS_WORD, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_STATUS_WORD]

    def setEncoderPolarityP(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, subid, 1)

    def setEncoderPolarityN(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, subid, -1)

    def getEncoderPolarity(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_POLARITY, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_ENCODER_POLARITY]

    def waitHomingDone(self, id, subid):
        condition = 1
        while condition:
            if (self.getStatus(id, subid) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0

    def waitTargetPositionReached(self, id, subid):
        condition = 1
        while condition:
            vel = self.getActualVelocity(id, subid)
            if (vel == 0):
                condition = 0

    def getDeviceID(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_DEVICE_ID, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_DEVICE_ID]

    def getDeviceType(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_BOARD_TYPE]

    def scanDevices(self):
        online = []
        print('Searching Online Devices...')
        for i in range(0, 32):
            self._motors[i][0][self._INDEX_BOARD_TYPE] = 0

        for i in range(0, 32):
            if self.getDeviceType(i, 0) != 0:
                online.append(i)

        print('Online Devices:')
        print(online)
        return online

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_MEMORY, id, 0, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_DEVICE_ID, id, 0, value)
        time.sleep(0.05)
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName):
        self._portName = portName
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        print('Pyhton SDK for DBD Ant Double Brushed DC Servo Motors Started')


class BeeLDCS:
    # Communication Profiles for BLDCS, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4
    _INDEX_TARGET_CURRENT = 5
    _INDEX_IO_OUT = 23
    _INDEX_KPP = 17
    _INDEX_KPI = 18
    _INDEX_KPD = 19
    _INDEX_KFF = 20
    _INDEX_PHASE_CORRECT_CURRENT = 21
    _INDEX_HOMING_DIRECTION = 14
    _INDEX_HOMING_LEVEL = 15
    _INDEX_ACC_TIME = 11
    _INDEX_SYNC_INTERPOLATION_TARGET_POSITION = 12
    _INDEX_TARGET_VELOCITY = 7
    _INDEX_TARGET_POSITION = 9
    _INDEX_ACTUAL_VELOCITY = 8
    _INDEX_ACTUAL_POSITION = 10
    _INDEX_IO_INPUT = 22
    _INDEX_ENCODER_OFFSET = 24
    _INDEX_ENCODER_POLARITY = 25
    _INDEX_ENCODER_VALUE = 26
    _INDEX_CURRENT_MAX = 28
    _INDEX_POSITION_ERROR_MAX = 29

    _FUNC_WRITE = 1
    _FUNC_READ = 0
    _FUNC_WRITE_OK = 3
    _FUNC_READ_OK = 2
    _FUNC_OPERATION = 4
    _FUNC_OPERATION_OK = 5
    _FUNC_CHECK = 254
    _FUNC_FREE = 255
    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_HOMING = 40
    _OPERATION_SYNC_INTERPOLATION_POSITION = 34
    _OPERATION_INDEX_MEMORY = 1
    _OPERATION_INDEX_TUNING = 2
    _STATUS_DEVICE_ENABLE = 0X01
    _STATUS_HOMG_FIND = 0X02
    _STATUS_TARGET_REACHED = 0X04
    _STATUS_IO_INPUT = 0X08
    _STATUS_ERROR_OVERCURRENT = 0X20
    _STATUS_ERROR_OVERPOSITION = 0X40
    _BOARD_TYPE_STEPPER_ANT = 0X10
    _BOARD_TYPE_STEPPER_BEE = 0X11
    _BOARD_TYPE_STEPPER_ELEPHANT = 0X12
    _BOARD_TYPE_BDCS_BEE = 0X13
    _BOARD_TYPE_BDC_BEE = 0X14
    _BOARD_TYPE_BLDCS_BEE = 0X15

    # Variables
    _connection = 0
    # Motors consist of 128 motor cells, each motor has 1024 parameters
    # _motors = numpy.zeros((32, 8, 32), dtype=numpy.int32)
    _array = (((ctypes.c_int32 * 32) * 8) * 32)
    _motors = _array()
    _sitp_msg = ctypes.create_string_buffer(12)
    _thread1 = 0
    _thread_stop_flag = 0
    _tx_queue = queue.Queue()
    _tx_queue_pdo = queue.Queue()
    _tx_sitp_queue = queue.Queue()  # sitp-> sync interpolation target position
    _tx_lock = threading.Lock()
    _rx_lock = threading.Lock()
    _tx_lock_pdo = threading.Lock()
    _rx_lock_pdo = threading.Lock()
    _sitp_buf_status = 0
    _sitp_flag = 0
    _tx_message = ctypes.create_string_buffer(8)
    rxmsg = ctypes.create_string_buffer(8)
    tempC = 0
    _private_msg = ctypes.create_string_buffer(8)
    _empty_msg = ctypes.create_string_buffer(8)
    func_code = 0
    index = 0
    id = 0
    subid = 0
    data = 0
    _errorFlag = 0

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, 2000000, timeout=1)

    # Disconnect form the serial port and close the communication
    def _disconnect(self):
        self._thread_stop_flag = 1
        self._connection.close()

    def _run(self):
        print("system run progress")

    # Analysis the Rx Message and Put the parameters into Each Motor
    # Note: need alarm message
    def _analysis(self, rx_message):
        ret = 0
        if len(rx_message) == 8:
            self._rx_lock.acquire()
            self.func_code = struct.unpack_from('B', rx_message, 0)
            self.index = struct.unpack_from('B', rx_message, 1)
            self.id = struct.unpack_from('B', rx_message, 2)
            self.subid = struct.unpack_from('B', rx_message, 3)
            self.data = struct.unpack_from('i', rx_message, 4)

            if self.func_code[0] == self._FUNC_READ_OK:
                ret = 1
                self._motors[self.id[0]][self.subid[0]][self.index[0]] = self.data[0]
                if self.index[0] == self._INDEX_STATUS_WORD:
                    if self.data[0] & self._STATUS_ERROR_OVERCURRENT == self._STATUS_ERROR_OVERCURRENT:
                        print("Error Over Current")
                        self._errorFlag = 1
                        # self.stop()
                        # exit(0)
                    if self.data[0] & self._STATUS_ERROR_OVERPOSITION == self._STATUS_ERROR_OVERPOSITION:
                        print("Error Over Position")
                        self._errorFlag = 1
                        # self.stop()
                        # exit(0)
            if self.func_code[0] == self._FUNC_WRITE_OK:
                ret = 1

            if self.func_code[0] == self._FUNC_FREE:
                ret = 1

            if self.func_code[0] == self._FUNC_CHECK:
                self._sitp_buf_status = self.data[0]
                ret = 1

            self._rx_lock.release()
        else:
            print("rx message error")
            ret = 0

        return ret

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        retransmitCounter = 0
        idleFlag = 0
        # msg = ctypes.create_string_buffer(8)
        while self._thread_stop_flag == 0:
            if self._sitp_flag == 1:
                self._connection.write(self._private_msg)
                self._analysis(self._connection.read(8))
                self._connection.reset_input_buffer()
                while self._sitp_buf_status < 0x03:
                    if not self._tx_sitp_queue.empty():
                        self._sitp_msg = self._tx_sitp_queue.get()
                    self._connection.write(self._sitp_msg)
                    self._analysis(self._connection.read(8))
                    self._connection.reset_input_buffer()
            else:

                if not self._tx_queue_pdo.empty():
                    self._tx_lock_pdo.acquire()
                    msg = self._tx_queue_pdo.get()
                    self._tx_lock_pdo.release()
                    self._connection.write(msg)
                    self._analysis(self._connection.read(8))
                    self._connection.reset_input_buffer()
                else:
                    if retransmitCounter == 0:
                        if not self._tx_queue.empty():
                            self._tx_lock.acquire()
                            self._tx_message = self._tx_queue.get()
                            self._tx_lock.release()
                            idleFlag = 0
                        else:
                            idleFlag = 1
                    else:
                        idleFlag = 0

                    if idleFlag == 1:
                        time.sleep(0.01)
                    else:
                        self._connection.write(self._tx_message)
                        if (self._analysis(self._connection.read(8)) == 1):
                            retransmitCounter = 0
                        else:
                            retransmitCounter = retransmitCounter + 1
                            if retransmitCounter > self.retransmitLimit:
                                retransmitCounter = 0
                        self._connection.reset_input_buffer()

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

    # Send Sync Interpolation Target Position
    def _sendMessageSITP32(self, data):
        message = ctypes.create_string_buffer(12)
        # 1 device 4 bytes, 32 devices total 128 bytes
        for i in range(0, 3):
            struct.pack_into('i', message, i * 4, int(*(data[i],)))
        self._tx_sitp_queue.put(message)

    def _sendMessagePDO(self, func_code, index, data):
        message = ctypes.create_string_buffer(132)
        struct.pack_into('B', message, 0, *(func_code,))
        struct.pack_into('B', message, 1, *(index,))
        for i in range(0, 32):
            struct.pack_into('i', message, 4 + i * 4, int(*(data,)))
        self._tx_lock_pdo.acquire()
        self._tx_queue_pdo.put(message)
        # print('append new message')
        self._tx_lock_pdo.release()

    def _sendMessageBDO(self, index, data):
        message = ctypes.create_string_buffer(8)
        struct.pack_into('B', message, 0, *(self._FUNC_WRITE,))
        struct.pack_into('B', message, 1, *(index,))
        struct.pack_into('B', message, 2, *(32,))
        struct.pack_into('B', message, 3, *(0,))
        struct.pack_into('i', message, 4, int(*(data,)))
        self._tx_lock.acquire()
        self._tx_queue.put(message)
        # print('append new message')
        self._tx_lock.release()

    # Stop the communication
    def stop(self):
        time.sleep(1)
        self._thread_stop_flag = 1
        self._sitp_buf_status = 3
        print('Pyhton SDK for DBD BLDC Servo Motor Stopped')

    def setPowerOn(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 1)

    def setPowerOff(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)

    def setTargetVelocity(self, id, value):
        # Note: Unit: pulse/ms (51200 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 3000 is reasonable, higher speed will lose steps
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, id, 0, value)

    def setTargetPosition(self, id, value):
        # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_POSITION, id, 0, value)

    def setVelocityMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_VELOCITY)

    def setPositionMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_POSITION)

    def setHomingMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_HOMING)

    def setSyncInterpolationPositionMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0,
                          self._OPERATION_SYNC_INTERPOLATION_POSITION)

    def setKPP(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPP, id, 0, value)

    def setKPI(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPI, id, 0, value)

    def setKPD(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPD, id, 0, value)

    def setKFF(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KFF, id, 0, value)

    def setTargetCurrent(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_CURRENT, id, 0, value)

    def setBoardTypeBLDCS(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_BOARD_TYPE, id, 0, self._BOARD_TYPE_BLDCS_BEE)

    def setPhaseCorrectCurrent(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_PHASE_CORRECT_CURRENT, id, 0, value)

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

    def setAccTime(self, id, value):
        # Note: acc time is a parameter for accelation and deaccelation progress, unit is ms, normally 200ms to 1000ms is reasonable
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ACC_TIME, id, 0, value)

    def setCurrentMax(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CURRENT_MAX, id, 0, value)

    def setPositionErrorMax(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_POSITION_ERROR_MAX, id, 0, value)

    # PDO Methods
    def setOutputIOPDO(self, value):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_IO_OUT, value)

    def setAccTimePDO(self, value):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_ACC_TIME, value)

    def setTargetVelocityPDO(self, value):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, value)

    def setTargetPositionPDO(self, value):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_TARGET_POSITION, value)

    def setHomingModePDO(self):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, self._OPERATION_MODE_HOMING)

    def setPositionModePDO(self):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, self._OPERATION_MODE_PROFILE_POSITION)

    def setPowerOnPDO(self):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, 1)

    def setPowerOffPDO(self):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, 0)

    # End of PDO Methods

    # BDO Methods (Broad Cast Message)
    def setPowerOnBDO(self):
        self._sendMessageBDO(self._INDEX_CONTROL_WORD, 1)

    def setPowerOffBDO(self):
        self._sendMessageBDO(self._INDEX_CONTROL_WORD, 0)

    def setOutputIOBDO(self, value):
        self._sendMessageBDO(self._INDEX_IO_OUT, value)

    # End of BDO Methods

    def getBoardType(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        time.sleep(0.05)
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BLDCS_BEE):
            print("Brushless DC Servo Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BDC_BEE):
            print("Brushed DC Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BDCS_BEE):
            print("Brushed DC Servo Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_STEPPER_BEE):
            print("Stepper Bee")
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def getAccTime(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACC_TIME, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ACC_TIME]

    def getADC(self,channel):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_POSITION, 4, channel, 0)
        time.sleep(0.05)
        return self._motors[4][channel][self._INDEX_ACTUAL_POSITION]

    def setOutputIO(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_IO_OUT, id, 0, value)

    def getHomingLevel(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_LEVEL, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_HOMING_LEVEL]

    def getHomingDirection(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_DIRECTION, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_HOMING_DIRECTION]

    def getKPP(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPP, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_KPP]

    def getKPI(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPI, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_KPI]

    def getKPD(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPD, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_KPD]

    def getKFF(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KFF, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_KFF]

    def getEncoderValue(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_VALUE, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ENCODER_VALUE]

    def getTargetCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_CURRENT, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_TARGET_CURRENT]

    def getInputIO(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_IO_INPUT, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_IO_INPUT]

    def getActualVelocity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_VELOCITY, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ACTUAL_VELOCITY]

    def getActualPosition(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_POSITION, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ACTUAL_POSITION]

    def getTargetVelocity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_VELOCITY, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_TARGET_VELOCITY]

    def getTargetPosition(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_POSITION, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_TARGET_POSITION]

    def getStatus(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_STATUS_WORD, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_STATUS_WORD]

    def setEncoderPolarityP(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, 0, 1)

    def setEncoderPolarityN(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, 0, -1)

    def getEncoderPolarity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_POLARITY, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ENCODER_POLARITY]

    def waitHomingDone(self, id):
        condition = 1
        while condition:
            if (self.getStatus(id) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0

    def waitTargetPositionReached(self, id):
        condition = 1
        self._motors[id][0][self._INDEX_STATUS_WORD] = 0
        while condition:
            st = self.getStatus(id)
            if (st & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0
            if self._errorFlag==1:
                condition = 0

    def getDeviceID(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_DEVICE_ID, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_DEVICE_ID]

    def getDeviceType(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        # if self._baudrate == 2000000:
        time.sleep(0.05)
        # else:
        #     time.sleep(0.2)
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def getPhaseCorrectCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_PHASE_CORRECT_CURRENT, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_PHASE_CORRECT_CURRENT]

    def getEncoderOffset(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_OFFSET, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ENCODER_OFFSET]

    def getCurrentMax(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_CURRENT_MAX, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_CURRENT_MAX]

    def getPositionErrorMax(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_POSITION_ERROR_MAX, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_POSITION_ERROR_MAX]

    def scanDevices(self):
        online = []
        self.retransmitLimit = 0
        print('Searching Online Devices...')
        for i in range(0, 32):
            self._motors[i][0][self._INDEX_BOARD_TYPE] = 0
        for i in range(0, 32):
            if self.getDeviceType(i) != 0:
                online.append(i)
        print('Online Devices:')
        self.retransmitLimit = 3
        print(online)

    def tunePhase(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_TUNING, id, 0, 1)

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_MEMORY, id, 0, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_DEVICE_ID, id, 0, value)
        time.sleep(0.05)
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName):
        struct.pack_into('B', self._private_msg, 0, *(self._FUNC_CHECK,))
        struct.pack_into('B', self._private_msg, 1, *(0,))
        struct.pack_into('B', self._private_msg, 2, *(0,))
        struct.pack_into('B', self._private_msg, 3, *(0,))
        struct.pack_into('i', self._private_msg, 4, int(*(0,)))
        self.func_code = 0
        self.index = 0
        self.id = 0
        self.subid = 0
        self.data = 0
        self.retransmitLimit = 3

        self._portName = portName
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        print('Pyhton SDK for DBD BLDC Servo Motor Started')


class BeeLED:
    # Communication Profiles, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4
    _INDEX_TARGET_CURRENT = 5
    _INDEX_COS_T = 7
    _INDEX_COS_A = 8
    _INDEX_TRIGGER_LEVEL = 9
    _INDEX_IO_OUT = 23
    _INDEX_IO_INPUT = 22
    _INDEX_PWM1 = 24
    _INDEX_PWM2 = 25
    _INDEX_PWM3 = 26
    _INDEX_PWM4 = 27
    _FUNC_WRITE = 1
    _FUNC_READ = 0
    _FUNC_WRITE_OK = 3
    _FUNC_READ_OK = 2
    _FUNC_OPERATION = 4
    _FUNC_OPERATION_OK = 5
    _FUNC_FREE = 255
    _OPERATION_MODE_PWM = 0
    _OPERATION_MODE_CURRENT = 10
    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_HOMING = 40
    _OPERATION_MODE_COS = 50
    _OPERATION_MODE_COS_TRIGGER = 51

    _OPERATION_INDEX_MEMORY = 1
    _OPERATION_INDEX_TUNING = 2
    _STATUS_DEVICE_ENABLE = 0X01
    _STATUS_HOMG_FIND = 0X02
    _STATUS_TARGET_REACHED = 0X04
    _STATUS_IO_INPUT = 0X08

    _BOARD_TYPE_STEPPER_ANT = 0X10
    _BOARD_TYPE_STEPPER_BEE = 0X11
    _BOARD_TYPE_STEPPER_ELEPHANT = 0X12
    _BOARD_TYPE_BDCS_BEE = 0X13
    _BOARD_TYPE_BDC_BEE = 0X14
    _BOARD_TYPE_BLDCS_BEE = 0X15

    # Variables
    _connection = 0
    # Motors consist of 128 motor cells, each motor has 1024 parameters
    # _motors = numpy.zeros((32, 8, 32), dtype=numpy.int32)
    array = (((ctypes.c_int32 * 32) * 8) * 32)
    _motors = array()
    _thread1 = 0
    _thread_stop_flag = 0
    _tx_queue = queue.Queue()

    # _tx_lock = threading.Lock()
    # _rx_lock = threading.Lock()

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, 2000000, timeout=0.1)

    # Disconnect form the serial port and close the communication
    def _disconnect(self):
        self._thread_stop_flag = 1
        self._connection.close()

    # Analysis the Rx Message and Put the parameters into Each Motor
    def _analysis(self, rx_message):
        if len(rx_message) == 8:
            # self._rx_lock.acquire()
            func_code = struct.unpack_from('B', rx_message, 0)
            index = struct.unpack_from('B', rx_message, 1)
            id = struct.unpack_from('B', rx_message, 2)
            subid = struct.unpack_from('B', rx_message, 3)
            data = struct.unpack_from('i', rx_message, 4)

            if func_code[0] == self._FUNC_READ_OK:
                self._motors[id[0]][subid[0]][index[0]] = data[0]

            # self._rx_lock.release()

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        while self._thread_stop_flag == 0:
            # while 1:
            # self._tx_lock.acquire()
            # print('msg length:', self._tx_queue.qsize())
            if not self._tx_queue.empty():
                msg = self._tx_queue.get()
                # self._tx_lock.release()
                self._connection.write(msg)
                self._analysis(self._connection.read(8))
                self._connection.reset_input_buffer()
            else:
                # self._tx_lock.release()
                time.sleep(0.01)

    # Send Message Function, Users would Call this Function to Send Messages
    def _sendMessage(self, func_code, index, id, subid, data):
        message = ctypes.create_string_buffer(8)
        struct.pack_into('B', message, 0, *(func_code,))
        struct.pack_into('B', message, 1, *(index,))
        struct.pack_into('B', message, 2, *(id,))
        struct.pack_into('B', message, 3, *(subid,))
        struct.pack_into('i', message, 4, int(*(data,)))
        # self._tx_lock.acquire()
        self._tx_queue.put(message)
        # print('append new message')
        # self._tx_lock.release()

    # Stop the communication
    def stop(self):
        time.sleep(1)
        self._thread_stop_flag = 1
        print('Pyhton SDK for DBD LED Controller Stopped')

    def setPowerOn(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 1)

    def setPowerOff(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)

    def setCurrentMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_CURRENT)

    def setCosMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_COS)

    def setCosT(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_COS_T, id, 0, value)

    def setCosA(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_COS_A, id, 0, value)

    def setTriggerLevel(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TRIGGER_LEVEL, id, 0, value)

    def setCosTriggerMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_COS_TRIGGER)

    def setTargetCurrent(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_CURRENT, id, 0, value)

    def setPWM1(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_PWM1, id, 0, value)

    def setPWM2(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_PWM2, id, 0, value)

    def setPWM3(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_PWM3, id, 0, value)

    def setPWM4(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_PWM4, id, 0, value)

    def getBoardType(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        time.sleep(0.05)
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BLDCS_BEE):
            print("Brushless DC Servo Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BDC_BEE):
            print("Brushed DC Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BDCS_BEE):
            print("Brushed DC Servo Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_STEPPER_BEE):
            print("Stepper Bee")
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def getTargetCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_CURRENT, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_TARGET_CURRENT]

    def getInputIO(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_IO_INPUT, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_IO_INPUT]

    def getStatus(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_STATUS_WORD, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_STATUS_WORD]

    def waitHomingDone(self, id):
        condition = 1
        while condition:
            if (self.getStatus(id) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0

    def waitTargetPositionReached(self, id):
        condition = 1
        while condition:
            if (self.getStatus(id) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0

    def getDeviceID(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_DEVICE_ID, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_DEVICE_ID]

    def getDeviceType(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        time.sleep(0.1)
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def scanDevices(self):
        online = []
        print('Searching Online Devices...')
        for i in range(0, 32):
            self._motors[i][0][self._INDEX_BOARD_TYPE] = 0
        for i in range(0, 32):
            if self.getDeviceType(i) != 0:
                online.append(i)
        print('Online Devices:')
        print(online)

    def tunePhase(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_TUNING, id, 0, 1)

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_MEMORY, id, 0, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_DEVICE_ID, id, 0, value)
        time.sleep(0.1)
        self.saveParameters(value)
        time.sleep(0.1)

    # Init Process
    def __init__(self, portName):
        self._portName = portName
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        print('Pyhton SDK for DBD Bee LED Controller Started')


class BeeEncoder:
    # Communication Profiles for BLDCS, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4
    _INDEX_ACTUAL_POSITION = 10
    _INDEX_ENCODER_VALUE = 26

    _FUNC_WRITE = 1
    _FUNC_READ = 0
    _FUNC_WRITE_OK = 3
    _FUNC_READ_OK = 2
    _FUNC_OPERATION = 4
    _FUNC_OPERATION_OK = 5
    _FUNC_CHECK = 254
    _FUNC_FREE = 255

    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_HOMING = 40
    _OPERATION_SYNC_INTERPOLATION_POSITION = 34
    _OPERATION_INDEX_MEMORY = 1
    _OPERATION_INDEX_TUNING = 2

    _STATUS_DEVICE_ENABLE = 0X01
    _STATUS_HOMG_FIND = 0X02
    _STATUS_TARGET_REACHED = 0X04
    _STATUS_IO_INPUT = 0X08
    _STATUS_ERROR_OVERCURRENT = 0X20
    _STATUS_ERROR_OVERPOSITION = 0X40

    _BOARD_TYPE_STEPPER_ANT = 0X10
    _BOARD_TYPE_STEPPER_BEE = 0X11
    _BOARD_TYPE_STEPPER_ELEPHANT = 0X12
    _BOARD_TYPE_BDCS_BEE = 0X13
    _BOARD_TYPE_BDC_BEE = 0X14
    _BOARD_TYPE_BLDCS_BEE = 0X15

    # Variables
    _connection = 0
    # Motors consist of 128 motor cells, each motor has 1024 parameters
    # _motors = numpy.zeros((32, 8, 32), dtype=numpy.int32)
    _array = (((ctypes.c_int32 * 32) * 8) * 32)
    _motors = _array()
    _encoders = (ctypes.c_int32 * 32)()
    _thread1 = 0
    _thread_stop_flag = 0
    _tx_queue = queue.Queue()
    _tx_queue_pdo = queue.Queue()
    _tx_sitp_queue = queue.Queue()  # sitp-> sync interpolation target position
    _tx_lock = threading.Lock()
    _rx_lock = threading.Lock()
    _tx_lock_pdo = threading.Lock()
    _rx_lock_pdo = threading.Lock()
    _sitp_buf_status = 0
    _sitp_flag = 0
    _tx_message = ctypes.create_string_buffer(32)
    rxmsg = ctypes.create_string_buffer(8)
    tempC = 0
    _private_msg = ctypes.create_string_buffer(8)
    _empty_msg = ctypes.create_string_buffer(8)
    func_code = 0
    index = 0
    id = 0
    subid = 0
    data = 0
    recordFlag = 0
    _record_queue = queue.Queue()

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, 2000000, timeout=1)

    # Disconnect form the serial port and close the communication
    def _disconnect(self):
        self._thread_stop_flag = 1
        self._connection.close()

    # Analysis the Rx Message and Put the parameters into Each Motor
    # Note: need alarm message
    def _analysis(self, rx_message):
        ret = 0
        if len(rx_message) == 8:
            self._rx_lock.acquire()
            self.func_code = struct.unpack_from('B', rx_message, 0)
            self.index = struct.unpack_from('B', rx_message, 1)
            self.id = struct.unpack_from('B', rx_message, 2)
            self.subid = struct.unpack_from('B', rx_message, 3)
            self.data = struct.unpack_from('i', rx_message, 4)

            if self.func_code[0] == self._FUNC_READ_OK:
                ret = 1
                self._motors[self.id[0]][self.subid[0]][self.index[0]] = self.data[0]

            if self.func_code[0] == self._FUNC_WRITE_OK:
                ret = 1

            if self.func_code[0] == self._FUNC_FREE:
                ret = 1

            if self.func_code[0] == self._FUNC_CHECK:
                self._sitp_buf_status = self.data[0]
                ret = 1

            self._rx_lock.release()
        else:
            print("rx message error")
            ret = 0

        return ret

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        # time_last = 0
        while self._thread_stop_flag == 0:

            struct.pack_into('B', self._tx_message, 0, *(0x20,))
            self._connection.write(self._tx_message)
            rx_message = self._connection.read(32)
            if len(rx_message) == 32:
                value = struct.unpack_from('B', rx_message, 0)
                if value[0] != 0:
                    # time_start = time.time()
                    # time_c = time_start - time_last
                    # print('time cost', time_c, 's')
                    # time_last = time_start
                    # print(value[0])
                    struct.pack_into('B', self._tx_message, 0, *(0x00,))
                    self._connection.write(self._tx_message)
                    enc0 = self._connection.read(32)

                    struct.pack_into('B', self._tx_message, 0, *(0x01,))
                    self._connection.write(self._tx_message)
                    enc1 = self._connection.read(32)

                    struct.pack_into('B', self._tx_message, 0, *(0x02,))
                    self._connection.write(self._tx_message)
                    enc2 = self._connection.read(32)

                    struct.pack_into('B', self._tx_message, 0, *(0x03,))
                    self._connection.write(self._tx_message)
                    enc3 = self._connection.read(32)

                    value = struct.unpack_from('8i', enc0, 0)
                    # print(value[0])
                    # print(value[1])
                    # print(value[2])
                    # print(value[3])
                    self._encoders[0] = value[0]
                    self._encoders[1] = value[1]
                    self._encoders[2] = value[2]
                    self._encoders[3] = value[3]
                    self._encoders[4] = value[4]
                    self._encoders[5] = value[5]
                    self._encoders[6] = value[6]
                    self._encoders[7] = value[7]

                    # value = struct.unpack_from('8i', enc1, 0)
                    # self._encoders[4] = value[0]
                    # self._encoders[5] = value[1]
                    # self._encoders[6] = value[2]
                    # self._encoders[7] = value[3]
                    #
                    # value = struct.unpack_from('8i', enc2, 0)
                    # self._encoders[8] = value[0]
                    # self._encoders[9] = value[1]
                    # self._encoders[10] = value[2]
                    # self._encoders[11] = value[3]
                    #
                    # value = struct.unpack_from('8i', enc3, 0)
                    # self._encoders[12] = value[0]
                    # self._encoders[13] = value[1]
                    # self._encoders[14] = value[2]
                    # self._encoders[15] = value[3]

                    if self.recordFlag == 1:
                        self._record_queue.put([value[0], value[1]])
                        # print(self._encoders[0])

            self._connection.reset_input_buffer()

    def recordStart(self):
        self.recordFlag = 1

    def recordStop(self):
        self.recordFlag = 0

    def recordClear(self):
        self._record_queue.empty()

    def recordSave(self, fileName):
        print('save')

    def recordPlot(self):
        size = self._record_queue.qsize()

        print(size)

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
        self._sitp_buf_status = 3
        print('Pyhton SDK for DBD BLDC Servo Motor Stopped')

    def getBoardType(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        time.sleep(0.05)
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BLDCS_BEE):
            print("Brushless DC Servo Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BDC_BEE):
            print("Brushed DC Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BDCS_BEE):
            print("Brushed DC Servo Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_STEPPER_BEE):
            print("Stepper Bee")
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def getDeviceID(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_DEVICE_ID, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_DEVICE_ID]

    def getDeviceType(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def scanDevices(self):
        online = []
        self.retransmitLimit = 0
        print('Searching Online Devices...')
        for i in range(0, 32):
            self._motors[i][0][self._INDEX_BOARD_TYPE] = 0
        for i in range(0, 32):
            if self.getDeviceType(i) != 0:
                online.append(i)
        print('Online Devices:')
        self.retransmitLimit = 3
        print(online)

    # Init Process
    def __init__(self, portName):
        struct.pack_into('B', self._private_msg, 0, *(self._FUNC_CHECK,))
        struct.pack_into('B', self._private_msg, 1, *(0,))
        struct.pack_into('B', self._private_msg, 2, *(0,))
        struct.pack_into('B', self._private_msg, 3, *(0,))
        struct.pack_into('i', self._private_msg, 4, int(*(0,)))
        self.func_code = 0
        self.index = 0
        self.id = 0
        self.subid = 0
        self.data = 0
        self.retransmitLimit = 3

        self._portName = portName
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        print('Pyhton SDK for DBD BLDC Servo Motor Started')


class BeeIO:
    # Communication Profiles for BLDCS, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4
    _INDEX_TARGET_CURRENT = 5
    _INDEX_IO_OUT = 23
    _INDEX_KPP = 17
    _INDEX_KPI = 18
    _INDEX_KPD = 19
    _INDEX_KFF = 20
    _INDEX_PHASE_CORRECT_CURRENT = 21
    _INDEX_HOMING_DIRECTION = 14
    _INDEX_HOMING_LEVEL = 15
    _INDEX_ACC_TIME = 11
    _INDEX_SYNC_INTERPOLATION_TARGET_POSITION = 12
    _INDEX_TARGET_VELOCITY = 7
    _INDEX_TARGET_POSITION = 9
    _INDEX_ACTUAL_VELOCITY = 8
    _INDEX_ACTUAL_POSITION = 10
    _INDEX_IO_INPUT = 22
    _INDEX_ENCODER_OFFSET = 24
    _INDEX_ENCODER_POLARITY = 25
    _INDEX_ENCODER_VALUE = 26
    _INDEX_CURRENT_MAX = 28
    _INDEX_POSITION_ERROR_MAX = 29

    _FUNC_WRITE = 1
    _FUNC_READ = 0
    _FUNC_WRITE_OK = 3
    _FUNC_READ_OK = 2
    _FUNC_OPERATION = 4
    _FUNC_OPERATION_OK = 5
    _FUNC_CHECK = 254
    _FUNC_FREE = 255
    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_HOMING = 40
    _OPERATION_SYNC_INTERPOLATION_POSITION = 34
    _OPERATION_INDEX_MEMORY = 1
    _OPERATION_INDEX_TUNING = 2
    _STATUS_DEVICE_ENABLE = 0X01
    _STATUS_HOMG_FIND = 0X02
    _STATUS_TARGET_REACHED = 0X04
    _STATUS_IO_INPUT = 0X08
    _STATUS_ERROR_OVERCURRENT = 0X20
    _STATUS_ERROR_OVERPOSITION = 0X40
    _BOARD_TYPE_STEPPER_ANT = 0X10
    _BOARD_TYPE_STEPPER_BEE = 0X11
    _BOARD_TYPE_STEPPER_ELEPHANT = 0X12
    _BOARD_TYPE_BDCS_BEE = 0X13
    _BOARD_TYPE_BDC_BEE = 0X14
    _BOARD_TYPE_BLDCS_BEE = 0X15

    # Variables
    _connection = 0
    # Motors consist of 128 motor cells, each motor has 1024 parameters
    # _motors = numpy.zeros((32, 8, 32), dtype=numpy.int32)
    _array = (((ctypes.c_int32 * 32) * 8) * 32)
    _motors = _array()
    _sitp_msg = ctypes.create_string_buffer(12)
    _thread1 = 0
    _thread_stop_flag = 0
    _tx_queue = queue.Queue()
    _tx_queue_pdo = queue.Queue()
    _tx_sitp_queue = queue.Queue()  # sitp-> sync interpolation target position
    _tx_lock = threading.Lock()
    _rx_lock = threading.Lock()
    _tx_lock_pdo = threading.Lock()
    _rx_lock_pdo = threading.Lock()
    _sitp_buf_status = 0
    _sitp_flag = 0
    _tx_message = ctypes.create_string_buffer(8)
    rxmsg = ctypes.create_string_buffer(8)
    tempC = 0
    _private_msg = ctypes.create_string_buffer(8)
    _empty_msg = ctypes.create_string_buffer(8)
    func_code = 0
    index = 0
    id = 0
    subid = 0
    data = 0

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, 2000000, timeout=1)

    # Disconnect form the serial port and close the communication
    def _disconnect(self):
        self._thread_stop_flag = 1
        self._connection.close()

    def _run(self):
        print("system run progress")

    # Analysis the Rx Message and Put the parameters into Each Motor
    # Note: need alarm message
    def _analysis(self, rx_message):
        ret = 0
        if len(rx_message) == 8:
            self._rx_lock.acquire()
            self.func_code = struct.unpack_from('B', rx_message, 0)
            self.index = struct.unpack_from('B', rx_message, 1)
            self.id = struct.unpack_from('B', rx_message, 2)
            self.subid = struct.unpack_from('B', rx_message, 3)
            self.data = struct.unpack_from('i', rx_message, 4)

            if self.func_code[0] == self._FUNC_READ_OK:
                ret = 1
                self._motors[self.id[0]][self.subid[0]][self.index[0]] = self.data[0]
                if self.index[0] == self._INDEX_STATUS_WORD:
                    if self.data[0] & self._STATUS_ERROR_OVERCURRENT == self._STATUS_ERROR_OVERCURRENT:
                        print("Error Over Current")
                        self.stop()
                        # exit(0)
                    if self.data[0] & self._STATUS_ERROR_OVERPOSITION == self._STATUS_ERROR_OVERPOSITION:
                        print("Error Over Position")
                        # self.stop()
                        # exit(0)
            if self.func_code[0] == self._FUNC_WRITE_OK:
                ret = 1

            if self.func_code[0] == self._FUNC_FREE:
                ret = 1

            if self.func_code[0] == self._FUNC_CHECK:
                self._sitp_buf_status = self.data[0]
                ret = 1

            self._rx_lock.release()
        else:
            print("rx message error")
            ret = 0

        return ret

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        retransmitCounter = 0
        idleFlag = 0
        # msg = ctypes.create_string_buffer(8)
        while self._thread_stop_flag == 0:
            if self._sitp_flag == 1:
                self._connection.write(self._private_msg)
                self._analysis(self._connection.read(8))
                self._connection.reset_input_buffer()
                while self._sitp_buf_status < 0x03:
                    if not self._tx_sitp_queue.empty():
                        self._sitp_msg = self._tx_sitp_queue.get()
                    self._connection.write(self._sitp_msg)
                    self._analysis(self._connection.read(8))
                    self._connection.reset_input_buffer()
            else:

                if not self._tx_queue_pdo.empty():
                    self._tx_lock_pdo.acquire()
                    msg = self._tx_queue_pdo.get()
                    self._tx_lock_pdo.release()
                    self._connection.write(msg)
                    self._analysis(self._connection.read(8))
                    self._connection.reset_input_buffer()
                else:
                    if retransmitCounter == 0:
                        if not self._tx_queue.empty():
                            self._tx_lock.acquire()
                            self._tx_message = self._tx_queue.get()
                            self._tx_lock.release()
                            idleFlag = 0
                        else:
                            idleFlag = 1
                    else:
                        idleFlag = 0

                    if idleFlag == 1:
                        time.sleep(0.01)
                    else:
                        self._connection.write(self._tx_message)
                        if (self._analysis(self._connection.read(8)) == 1):
                            retransmitCounter = 0
                        else:
                            retransmitCounter = retransmitCounter + 1
                            if retransmitCounter > self.retransmitLimit:
                                retransmitCounter = 0
                        self._connection.reset_input_buffer()

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

    # Send Sync Interpolation Target Position
    def _sendMessageSITP32(self, data):
        message = ctypes.create_string_buffer(12)
        # 1 device 4 bytes, 32 devices total 128 bytes
        for i in range(0, 3):
            struct.pack_into('i', message, i * 4, int(*(data[i],)))
        self._tx_sitp_queue.put(message)

    def _sendMessagePDO(self, func_code, index, data):
        message = ctypes.create_string_buffer(132)
        struct.pack_into('B', message, 0, *(func_code,))
        struct.pack_into('B', message, 1, *(index,))
        for i in range(0, 32):
            struct.pack_into('i', message, 4 + i * 4, int(*(data,)))
        self._tx_lock_pdo.acquire()
        self._tx_queue_pdo.put(message)
        # print('append new message')
        self._tx_lock_pdo.release()

    def _sendMessageBDO(self, index, data):
        message = ctypes.create_string_buffer(8)
        struct.pack_into('B', message, 0, *(self._FUNC_WRITE,))
        struct.pack_into('B', message, 1, *(index,))
        struct.pack_into('B', message, 2, *(32,))
        struct.pack_into('B', message, 3, *(0,))
        struct.pack_into('i', message, 4, int(*(data,)))
        self._tx_lock.acquire()
        self._tx_queue.put(message)
        # print('append new message')
        self._tx_lock.release()

    # Stop the communication
    def stop(self):
        time.sleep(1)
        self._thread_stop_flag = 1
        self._sitp_buf_status = 3
        print('Pyhton SDK for DBD BLDC Servo Motor Stopped')

    def setPowerOn(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 1)

    def setPowerOff(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)

    def setTargetVelocity(self, id, value):
        # Note: Unit: pulse/ms (51200 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 3000 is reasonable, higher speed will lose steps
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, id, 0, value)

    def setTargetPosition(self, id, value):
        # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_POSITION, id, 0, value)

    def setVelocityMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_VELOCITY)

    def setPositionMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_POSITION)

    def setHomingMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_HOMING)

    def setSyncInterpolationPositionMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0,
                          self._OPERATION_SYNC_INTERPOLATION_POSITION)

    def setKPP(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPP, id, 0, value)

    def setKPI(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPI, id, 0, value)

    def setKPD(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPD, id, 0, value)

    def setKFF(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KFF, id, 0, value)

    def setTargetCurrent(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_CURRENT, id, 0, value)

    def setBoardTypeBLDCS(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_BOARD_TYPE, id, 0, self._BOARD_TYPE_BLDCS_BEE)

    def setPhaseCorrectCurrent(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_PHASE_CORRECT_CURRENT, id, 0, value)

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

    def setAccTime(self, id, value):
        # Note: acc time is a parameter for accelation and deaccelation progress, unit is ms, normally 200ms to 1000ms is reasonable
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ACC_TIME, id, 0, value)

    def setCurrentMax(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CURRENT_MAX, id, 0, value)

    def setPositionErrorMax(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_POSITION_ERROR_MAX, id, 0, value)

    # PDO Methods
    def setOutputIOPDO(self, value):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_IO_OUT, value)

    def setAccTimePDO(self, value):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_ACC_TIME, value)

    def setTargetVelocityPDO(self, value):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, value)

    def setTargetPositionPDO(self, value):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_TARGET_POSITION, value)

    def setHomingModePDO(self):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, self._OPERATION_MODE_HOMING)

    def setPositionModePDO(self):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, self._OPERATION_MODE_PROFILE_POSITION)

    def setPowerOnPDO(self):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, 1)

    def setPowerOffPDO(self):
        self._sendMessagePDO(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, 0)

    # End of PDO Methods

    # BDO Methods (Broad Cast Message)
    def setPowerOnBDO(self):
        self._sendMessageBDO(self._INDEX_CONTROL_WORD, 1)

    def setPowerOffBDO(self):
        self._sendMessageBDO(self._INDEX_CONTROL_WORD, 0)

    def setOutputIOBDO(self, value):
        self._sendMessageBDO(self._INDEX_IO_OUT, value)

    # End of BDO Methods

    def getBoardType(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        time.sleep(0.05)
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BLDCS_BEE):
            print("Brushless DC Servo Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BDC_BEE):
            print("Brushed DC Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_BDCS_BEE):
            print("Brushed DC Servo Bee")
        if (self._motors[id][0][self._INDEX_BOARD_TYPE] == self._BOARD_TYPE_STEPPER_BEE):
            print("Stepper Bee")
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def getAccTime(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACC_TIME, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ACC_TIME]

    def setOutputIO(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_IO_OUT, id, 0, value)

    def getHomingLevel(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_LEVEL, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_HOMING_LEVEL]

    def getHomingDirection(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_DIRECTION, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_HOMING_DIRECTION]

    def getKPP(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPP, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_KPP]

    def getKPI(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPI, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_KPI]

    def getKPD(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPD, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_KPD]

    def getKFF(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KFF, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_KFF]

    def getEncoderValue(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_VALUE, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ENCODER_VALUE]

    def getTargetCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_CURRENT, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_TARGET_CURRENT]

    def getInputIO(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_IO_INPUT, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_IO_INPUT]

    def getActualVelocity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_VELOCITY, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ACTUAL_VELOCITY]

    def getActualPosition(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_POSITION, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ACTUAL_POSITION]

    def getTargetVelocity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_VELOCITY, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_TARGET_VELOCITY]

    def getTargetPosition(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_POSITION, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_TARGET_POSITION]

    def getStatus(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_STATUS_WORD, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_STATUS_WORD]

    def setEncoderPolarityP(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, 0, 1)

    def setEncoderPolarityN(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, 0, -1)

    def getEncoderPolarity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_POLARITY, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ENCODER_POLARITY]

    def waitHomingDone(self, id):
        condition = 1
        while condition:
            if (self.getStatus(id) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0

    def waitTargetPositionReached(self, id):
        condition = 1
        self._motors[id][0][self._INDEX_STATUS_WORD] = 0
        while condition:
            st = self.getStatus(id)
            # av = self.getActualVelocity(id)
            # tp = self.getTargetPosition(id)
            # ap = self.getActualPosition(id)

            if (st & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                # if av == 0:
                #     if tp == ap:
                condition = 0

    def getDeviceID(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_DEVICE_ID, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_DEVICE_ID]

    def getDeviceType(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def getPhaseCorrectCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_PHASE_CORRECT_CURRENT, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_PHASE_CORRECT_CURRENT]

    def getEncoderOffset(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_OFFSET, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_ENCODER_OFFSET]

    def getCurrentMax(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_CURRENT_MAX, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_CURRENT_MAX]

    def getPositionErrorMax(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_POSITION_ERROR_MAX, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_POSITION_ERROR_MAX]

    def scanDevices(self):
        online = []
        self.retransmitLimit = 0
        print('Searching Online Devices...')
        for i in range(0, 32):
            self._motors[i][0][self._INDEX_BOARD_TYPE] = 0
        for i in range(0, 32):
            if self.getDeviceType(i) != 0:
                online.append(i)
        print('Online Devices:')
        self.retransmitLimit = 3
        print(online)

    def tunePhase(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_TUNING, id, 0, 1)

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_MEMORY, id, 0, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_DEVICE_ID, id, 0, value)
        time.sleep(0.05)
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName):
        struct.pack_into('B', self._private_msg, 0, *(self._FUNC_CHECK,))
        struct.pack_into('B', self._private_msg, 1, *(0,))
        struct.pack_into('B', self._private_msg, 2, *(0,))
        struct.pack_into('B', self._private_msg, 3, *(0,))
        struct.pack_into('i', self._private_msg, 4, int(*(0,)))
        self.func_code = 0
        self.index = 0
        self.id = 0
        self.subid = 0
        self.data = 0
        self.retransmitLimit = 3

        self._portName = portName
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        print('Pyhton SDK for DBD BLDC Servo Motor Started')


class LED2812:
    # Communication Profiles for BLDCS, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4
    _INDEX_TARGET_CURRENT = 5
    _INDEX_IO_OUT = 23
    _INDEX_KPP = 17
    _INDEX_KPI = 18
    _INDEX_KPD = 19
    _INDEX_KFF = 20
    _INDEX_PHASE_CORRECT_CURRENT = 21
    _INDEX_HOMING_DIRECTION = 14
    _INDEX_HOMING_LEVEL = 15
    _INDEX_ACC_TIME = 11
    _INDEX_SYNC_INTERPOLATION_TARGET_POSITION = 12
    _INDEX_TARGET_VELOCITY = 7
    _INDEX_TARGET_POSITION = 9
    _INDEX_ACTUAL_VELOCITY = 8
    _INDEX_ACTUAL_POSITION = 10
    _INDEX_IO_INPUT = 22
    _INDEX_ENCODER_OFFSET = 24
    _INDEX_ENCODER_POLARITY = 25
    _INDEX_ENCODER_VALUE = 26
    _INDEX_CURRENT_MAX = 28
    _INDEX_POSITION_ERROR_MAX = 29

    _FUNC_WRITE = 1
    _FUNC_READ = 0
    _FUNC_WRITE_OK = 3
    _FUNC_READ_OK = 2
    _FUNC_OPERATION = 4
    _FUNC_OPERATION_OK = 5
    _FUNC_CHECK = 254
    _FUNC_FREE = 255
    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_HOMING = 40
    _OPERATION_SYNC_INTERPOLATION_POSITION = 34
    _OPERATION_INDEX_MEMORY = 1
    _OPERATION_INDEX_TUNING = 2
    _STATUS_DEVICE_ENABLE = 0X01
    _STATUS_HOMG_FIND = 0X02
    _STATUS_TARGET_REACHED = 0X04
    _STATUS_IO_INPUT = 0X08
    _STATUS_ERROR_OVERCURRENT = 0X20
    _STATUS_ERROR_OVERPOSITION = 0X40
    _BOARD_TYPE_STEPPER_ANT = 0X10
    _BOARD_TYPE_STEPPER_BEE = 0X11
    _BOARD_TYPE_STEPPER_ELEPHANT = 0X12
    _BOARD_TYPE_BDCS_BEE = 0X13
    _BOARD_TYPE_BDC_BEE = 0X14
    _BOARD_TYPE_BLDCS_BEE = 0X15

    # Variables
    _connection = 0
    # Motors consist of 128 motor cells, each motor has 1024 parameters
    # _motors = numpy.zeros((32, 8, 32), dtype=numpy.int32)
    _array = (((ctypes.c_int32 * 32) * 8) * 32)
    _motors = _array()
    _sitp_msg = ctypes.create_string_buffer(12)
    _thread1 = 0
    _thread_stop_flag = 0
    _tx_queue = queue.Queue()
    _tx_queue_pdo = queue.Queue()
    _tx_sitp_queue = queue.Queue()  # sitp-> sync interpolation target position
    _tx_lock = threading.Lock()
    _rx_lock = threading.Lock()
    _tx_lock_pdo = threading.Lock()
    _rx_lock_pdo = threading.Lock()
    _sitp_buf_status = 0
    _sitp_flag = 0
    _tx_message = ctypes.create_string_buffer(8)
    rxmsg = ctypes.create_string_buffer(8)
    tempC = 0
    _private_msg = ctypes.create_string_buffer(8)
    _empty_msg = ctypes.create_string_buffer(8)
    func_code = 0
    index = 0
    id = 0
    subid = 0
    data = 0

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, 115200, timeout=1)

    # Disconnect form the serial port and close the communication
    def _disconnect(self):
        self._thread_stop_flag = 1
        self._connection.close()

    # Analysis the Rx Message and Put the parameters into Each Motor
    # Note: need alarm message
    def _analysis(self, rx_message):
        ret = 0
        if len(rx_message) == 8:
            self._rx_lock.acquire()
            self.func_code = struct.unpack_from('B', rx_message, 0)
            self.index = struct.unpack_from('B', rx_message, 1)
            self.id = struct.unpack_from('B', rx_message, 2)
            self.subid = struct.unpack_from('B', rx_message, 3)
            self.data = struct.unpack_from('i', rx_message, 4)

            if self.func_code[0] == self._FUNC_READ_OK:
                ret = 1
                self._motors[self.id[0]][self.subid[0]][self.index[0]] = self.data[0]
                if self.index[0] == self._INDEX_STATUS_WORD:
                    if self.data[0] & self._STATUS_ERROR_OVERCURRENT == self._STATUS_ERROR_OVERCURRENT:
                        print("Error Over Current")
                        self.stop()
                        # exit(0)
                    if self.data[0] & self._STATUS_ERROR_OVERPOSITION == self._STATUS_ERROR_OVERPOSITION:
                        print("Error Over Position")
                        # self.stop()
                        # exit(0)
            if self.func_code[0] == self._FUNC_WRITE_OK:
                ret = 1

            if self.func_code[0] == self._FUNC_FREE:
                ret = 1

            if self.func_code[0] == self._FUNC_CHECK:
                self._sitp_buf_status = self.data[0]
                ret = 1

            self._rx_lock.release()
        else:
            print("rx message error")
            ret = 0

        return ret

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        retransmitCounter = 0
        # msg = ctypes.create_string_buffer(8)
        while self._thread_stop_flag == 0:

            if not self._tx_queue.empty():
                self._tx_lock.acquire()
                self._tx_message = self._tx_queue.get()
                self._tx_lock.release()

                self._connection.write(self._tx_message)

                self._connection.reset_input_buffer()
            else:
                time.sleep(0.05)

    # Send Message Function, Users would Call this Function to Send Messages
    # DD 55 EE 00 00 00 01 00 99 01  00 00 00 03 00 01 FF 00 80 AA BB
    def _sendMessage(self, rgb):
        message = ctypes.create_string_buffer(234)
        struct.pack_into('B', message, 0, 0xdd)
        struct.pack_into('B', message, 1, 0x55)
        struct.pack_into('B', message, 2, 0xEE)
        struct.pack_into('B', message, 3, 0x00)
        struct.pack_into('B', message, 4, 0x00)
        struct.pack_into('B', message, 5, 0x00)
        struct.pack_into('B', message, 6, 0x01)
        struct.pack_into('B', message, 7, 0x00)
        struct.pack_into('B', message, 8, 0x99)
        struct.pack_into('B', message, 9, 0x01)
        struct.pack_into('B', message, 10, 0x00)
        struct.pack_into('B', message, 11, 0x00)
        struct.pack_into('B', message, 12, 0x00)
        struct.pack_into('B', message, 13, 3*72)
        struct.pack_into('B', message, 14, 0x00)
        struct.pack_into('B', message, 15, 0x01)
        # 1
        for i in range(0, 72*3):
            struct.pack_into('B', message, 16+i, *(rgb[i],))

        struct.pack_into('B', message, 16+216, 0xAA)
        struct.pack_into('B', message, 17+216, 0xBB)
        self._tx_lock.acquire()
        self._tx_queue.put(message)
        # print('append new message')
        self._tx_lock.release()
    # Stop the communication
    def stop(self):
        time.sleep(0.5)
        self._thread_stop_flag = 1
        print('Python SDK for DBD Ant Stopped.')

    # Init Process
    def __init__(self, portName):
        struct.pack_into('B', self._private_msg, 0, *(self._FUNC_CHECK,))
        struct.pack_into('B', self._private_msg, 1, *(0,))
        struct.pack_into('B', self._private_msg, 2, *(0,))
        struct.pack_into('B', self._private_msg, 3, *(0,))
        struct.pack_into('i', self._private_msg, 4, int(*(0,)))
        self.func_code = 0
        self.index = 0
        self.id = 0
        self.subid = 0
        self.data = 0
        self.retransmitLimit = 3

        self._portName = portName
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        print('Pyhton SDK for DBD LED 2812 Started')