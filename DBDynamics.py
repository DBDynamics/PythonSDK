#!/usr/bin/python3

import ctypes
import queue
import struct
import threading
import time

import numpy
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
    _SUBINDEX_WRITE = 0
    _SUBINDEX_READ = 1
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
    def _analysis(self, rx_message):
        if len(rx_message) == 11:
            self._rx_lock.acquire()
            id = struct.unpack_from('h', rx_message, 0)
            index = struct.unpack_from('h', rx_message, 2)
            subindex = struct.unpack_from('h', rx_message, 4)
            data = struct.unpack_from('i', rx_message, 6)
            size = struct.unpack_from('B', rx_message, 10)
            func_code = id[0] & 0xff80
            if func_code != self._FUNC_CODE_FREE:
                device_id = id[0] & 0x007f
                self._motors[device_id][index[0]] = data[0]
            self._rx_lock.release()

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        while self._thread_stop_flag == 0:
            self._tx_lock.acquire()
            # print('msg length:', self._tx_queue.qsize())
            if not self._tx_queue.empty():
                msg = self._tx_queue.get()
                self._tx_lock.release()
                self._connection.write(msg)
                self._analysis(self._connection.read(11))
            else:
                # msg = ctypes.create_string_buffer(10)
                # struct.pack_into('h', msg, 0, self._FUNC_CODE_FREE)
                # struct.pack_into('h', msg, 2, *(0,))
                # struct.pack_into('h', msg, 4, *(0,))
                # struct.pack_into('i', msg, 6, *(0,))
                self._tx_lock.release()
                time.sleep(0.01)

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

    def getDeviceID(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_DEVICE_ID, self._SUBINDEX_READ, 0)
        time.sleep(0.05)
        return self._motors[id][self._INDEX_DEVICE_ID]

    def scanDevices(self):
        online = []
        print('Searching Online Devices...')
        for i in range(1, 121):
            if i == self.getDeviceID(i):
                online.append(i)
        print('Online Devices:')
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
        self._motors = numpy.zeros((128, 1024), dtype=numpy.int32)
        self._thread_stop_flag = 0
        self._tx_queue = queue.Queue()
        self._tx_lock = threading.Lock()
        self._rx_lock = threading.Lock()
        self._portName = portName
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

    # Variables
    _connection = 0
    # Motors consist of 128 motor cells, each motor has 1024 parameters
    _motors = numpy.zeros((32, 8, 32), dtype=numpy.int32)
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

    def getRunningCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_RUNNING_CURRENT, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_RUNNING_CURRENT]

    def getKeepingCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KEEPING_CURRENT, id, 0, 0)
        time.sleep(0.05)
        return self._motors[id][0][self._INDEX_KEEPING_CURRENT]

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

    def waitHomingDone(self, id):
        condition = 1
        while condition:
            vel = self.getActualVelocity(id)
            if vel == 0:
                condition = 0

    def waitTargetPositionReached(self, id):
        condition = 1
        while condition:
            vel = self.getActualVelocity(id)
            if vel == 0:
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
            if self.getDeviceType(i) != 0:
                online.append(i)
        print('Online Devices:')
        print(online)

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_MEMORY, id, 0, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_READ, self._INDEX_DEVICE_ID, id, 0, value)
        time.sleep(0.05)
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName):
        self._portName = portName
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
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
    _INDEX_IO_INPUT = 22
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

    # Variables
    _connection = 0
    # Motors consist of 128 motor cells, each motor has 1024 parameters
    _motors = numpy.zeros((32, 8, 32), dtype=numpy.int32)
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

    def setAccTime(self, id, subid, value):
        # Note: acc time is a parameter for accelation and deaccelation progress, unit is ms, normally 200ms to 1000ms is reasonable
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ACC_TIME, id, subid, value)

    def getAccTime(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACC_TIME, id, subid, 0)
        time.sleep(0.05)
        return self._motors[id][subid][self._INDEX_ACC_TIME]

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
        self._thread1.start()
        print('Pyhton SDK for DBD Elephant Started')


class BeeDCS:
    # Communication Profiles, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4
    _INDEX_TARGET_CURRENT = 5;
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
    _motors = numpy.zeros((32, 8, 32), dtype=numpy.int32)
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
            vel = self.getActualVelocity(id)
            if (vel == 0):
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
            if self.getDeviceType(i) != 0:
                online.append(i)
        print('Online Devices:')
        print(online)

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_MEMORY, id, 0, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_READ, self._INDEX_DEVICE_ID, id, 0, value)
        time.sleep(0.05)
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName):
        self._portName = portName
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        print('Pyhton SDK for DBD Bee Brushed DC Servo Motor Started')
