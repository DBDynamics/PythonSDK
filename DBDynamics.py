#!/usr/bin/python3

import ctypes
import queue
import struct
import threading
import time
import serial
from scipy.interpolate import PchipInterpolator


# import usb
# import usb.backend.libusb1


# Class for basic communication
# Each Stepper Group consist of 128 motors
class Ant:
    # Communication Profiles, do not change them!
    _INDEX_CONTROL_WORD = 0
    _INDEX_OPERATION_MODE = 1
    _INDEX_IO_OUT = 14
    _INDEX_MEMORY = 30
    _INDEX_DEVICE_ID = 31
    _INDEX_SYNC_TARGET_POSITION = 38
    _INDEX_SYNC_RUN = 39
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
    _INDEX_HOMING_OFFSET = 60

    _INDEX_TX_FEEDBACK = 96
    _INDEX_SINGLE_DIRECTION = 97
    _SUBINDEX_WRITE = 0
    _SUBINDEX_READ = 1
    _SUBINDEX_WRITE_OK = 2
    _SUBINDEX_READ_OK = 3
    _OPERATION_MODE_PROFILE_VELOCITY = 21
    _OPERATION_MODE_PROFILE_POSITION = 31
    _OPERATION_MODE_PROFILE_POSITION_SYNC = 33

    _OPERATION_MODE_HOMING = 40
    _OPERATION_MODE_INTERPOLATION = 34
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
        ret = 0
        if len(rx_message) == 11:
            # print(rx_message)
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
                                # print(index[0])
                                self._motors[device_id][index[0]] = data[0]
                                ret = 1
                            elif subindex[0] == self._SUBINDEX_WRITE_OK:
                                ret = 1
            else:
                ret = 1
        return ret

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        while self._thread_stop_flag == 0:
            if self._broadcast_mode == 0:
                self._tx_lock.acquire()
                # print('msg length:', self._tx_queue.qsize())
                if self._retransmitCounter == 0:
                    if not self._tx_queue.empty():
                        self._msg = self._tx_queue.get()
                    else:
                        struct.pack_into('h', self._msg, 0, self._FUNC_CODE_FREE)
                        struct.pack_into('h', self._msg, 2, *(0,))
                        struct.pack_into('h', self._msg, 4, *(0,))
                        struct.pack_into('i', self._msg, 6, *(0,))

                self._tx_lock.release()
                # time.sleep(0.01)

                self._connection.write(self._msg)
                self._analysis(self._connection.read(11))

                self._connection.reset_input_buffer()
            elif self._broadcast_mode == 1:
                # time_start = time.time()
                self._connection.write(self._broadcast_msg)
                self._connection.read(11)
                #
                self._connection.reset_input_buffer()
                # time_end = time.time()
                # time_c = time_end - time_start
                # print('time cost', time_c*1000, 'ms')
                time.sleep(0.06)

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

    def getXXX(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, 201, self._SUBINDEX_READ, 0)
        self._sendMessage(self._FUNC_CODE_TSDO, id, 202, self._SUBINDEX_READ, 0)
        self._sendMessage(self._FUNC_CODE_TSDO, id, 203, self._SUBINDEX_READ, 0)
        self._sendMessage(self._FUNC_CODE_TSDO, id, 204, self._SUBINDEX_READ, 0)

    def setXXX(self, id, sn, s1, s2, s3, s4):
        self._sendMessage(self._FUNC_CODE_TSDO, id, 204, self._SUBINDEX_WRITE, sn)
        self._sendMessage(self._FUNC_CODE_TSDO, id, 205, self._SUBINDEX_WRITE, s1)
        self._sendMessage(self._FUNC_CODE_TSDO, id, 206, self._SUBINDEX_WRITE, s2)
        self._sendMessage(self._FUNC_CODE_TSDO, id, 207, self._SUBINDEX_WRITE, s3)
        self._sendMessage(self._FUNC_CODE_TSDO, id, 208, self._SUBINDEX_WRITE, s4)

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
        # Disables power to the motor with the specified ID by sending a control word with value 0
        # Parameters:
        #   id: The ID of the motor to turn off
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_CONTROL_WORD, self._SUBINDEX_WRITE, 0)

    def setTargetVelocity(self, id, value):
        # Note: Unit: pulse/ms (50000 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 300 is reasonable, higher speed will lose steps
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_VELOCITY, self._SUBINDEX_WRITE, value)

    def setTargetPosition(self, id, value):
        # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_POSITION, self._SUBINDEX_WRITE, value)

    def setVelocityMode(self, id):
        # Sets the motor to velocity control mode, allowing direct speed control
        # Parameters:
        #   id: The ID of the motor to configure
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
                          self._OPERATION_MODE_PROFILE_VELOCITY)

    def setPositionMode(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
                          self._OPERATION_MODE_PROFILE_POSITION)

    def setSyncPositionMode(self, id):
        # Sets the motor to synchronized position control mode, allowing coordinated motion
        # between multiple motors. Use this mode before calling setSyncTargetPosition()
        # Parameters:
        #   id: The ID of the motor to configure
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
                          self._OPERATION_MODE_PROFILE_POSITION_SYNC)

    def setSyncTargetPosition(self, id, value):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_SYNC_TARGET_POSITION, self._SUBINDEX_WRITE, value)

    def setSyncRun(self):
        self._sendMessage(self._FUNC_CODE_SYNC, 0, self._INDEX_SYNC_RUN, self._SUBINDEX_WRITE, 0)

    def setHomingMode(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
                          self._OPERATION_MODE_HOMING)

    def setHomingOffset(self, id, value):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_OFFSET, self._SUBINDEX_WRITE,
                          value)

    def setInterpolationMode(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
                          self._OPERATION_MODE_INTERPOLATION)

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

    def waitTargetPositionReached(self, id):
        condition = 1
        while condition:
            vel = self.getActualVelocity(id)
            if vel == 0:
                condition = 0

    def waitHomingDone(self, id):
        condition = 1
        while condition:
            pos = self.getActualPosition(id)
            print(pos)
            if pos == 0:
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
        self._retransmitLimit = 0
        print('Searching Online Devices...')
        for i in range(0, 121):
            self._motors[i][self._INDEX_DEVICE_ID] = 0
        for i in range(1, 121):
            if i == self.getDeviceID(i):
                online.append(i)
        print('Online Devices:')
        self._retransmitLimit = 0
        print(online)

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_MEMORY, self._SUBINDEX_WRITE, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_DEVICE_ID, self._SUBINDEX_WRITE, value)
        time.sleep(0.05)
        self.saveParameters(value)

    def setBroadCastMode(self, status):
        if status:
            print("Enter Message Broadcast Mode")
            self._broadcast_mode = 1
        else:
            print("Exit Message Broadcast Mode")
            self._broadcast_mode = 0

    def setBroadcastPosition(self):
        self._broadcast_msg[0] = 0x01
        self._broadcast_msg[1] = self._INDEX_TARGET_POSITION
        self._broadcast_msg[2] = 0x00
        self._broadcast_msg[3] = 0x00
        for i in range(1, 121):
            struct.pack_into('i', self._broadcast_msg, i * 4, *(self.broadcast_position[i],))

    def setBroadcastPowerOn(self):
        self._broadcast_msg[0] = 0x01
        self._broadcast_msg[1] = self._INDEX_CONTROL_WORD
        self._broadcast_msg[2] = 0x00
        self._broadcast_msg[3] = 0x00
        struct.pack_into('120i', self._broadcast_msg, 4, *(1,) * 120)
        time.sleep(0.1)

    def setBroadcastPowerOff(self):
        self._broadcast_msg[0] = 0x01
        self._broadcast_msg[1] = self._INDEX_CONTROL_WORD
        self._broadcast_msg[2] = 0x00
        self._broadcast_msg[3] = 0x00
        struct.pack_into('120i', self._broadcast_msg, 4, *(0,) * 120)
        time.sleep(0.1)

    # Init Process
    def __init__(self, portName):
        # Variables
        self._connection = 0
        # Motors consist of 128 motor cells, each motor has 1024 parameters
        # self._motors = numpy.zeros((128, 1024), dtype=numpy.int32)
        array = (ctypes.c_int32 * 1024 * 128)
        self._motors = array()
        pos_array = (ctypes.c_int32 * 128)
        self.broadcast_position = pos_array()
        self._thread_stop_flag = 0
        self._retransmitCounter = 0
        self._retransmitLimit = 10
        self._tx_queue = queue.Queue()
        self._tx_lock = threading.Lock()
        self._rx_lock = threading.Lock()
        self._broadcast_mode = 0
        self._portName = portName
        self._msg = ctypes.create_string_buffer(10)
        self._broadcast_msg = ctypes.create_string_buffer(484)
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


class BeeS:
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
    _INDEX_HOMING_TRIGGER = 13
    _INDEX_HOMING_DIRECTION = 14
    _INDEX_HOMING_LEVEL = 15
    _INDEX_ACC_TIME = 11
    _INDEX_ACTUAL_CURRENT = 6
    _INDEX_TARGET_VELOCITY = 7
    _INDEX_TARGET_POSITION = 9
    _INDEX_ACTUAL_VELOCITY = 8
    _INDEX_ACTUAL_POSITION = 10
    _INDEX_IO_INPUT = 22
    _INDEX_POWER_LIMIT = 28

    _INDEX_KPP = 17
    _INDEX_KPI = 18
    _INDEX_KVF = 19
    _INDEX_KFF = 20
    _INDEX_LIMIT_POSITION_P = 36
    _INDEX_LIMIT_POSITION_N = 37

    # for state machine 2
    _INDEX_TP0 = 25
    _INDEX_TP1 = 26

    _INDEX_ENCODER_VALUE = 26
    _INDEX_ENCODER_POLARITY = 25
    _INDEX_ENCODER_OFFSET = 24
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
    _INDEX_ESTOP_DEC = 40

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
    _OPERATION_INDEX_TUNING = 2
    _STATUS_DEVICE_ENABLE = 0X01
    _STATUS_HOMG_FIND = 0X02
    _STATUS_TARGET_REACHED = 0X04
    _STATUS_IO_INPUT = 0X08
    _STATUS_IO_LIMIT_P = 0X10
    _STATUS_IO_LIMIT_N = 0X20
    _STATUS_ESTOP = 0X40

    _INDEX_PHASE_CORRECT_CURRENT = 21
    _INDEX_HOMING_TRIGGER = 14

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
                if subid[0] > 0:
                    self._error_axis_num = id[0]
                else:
                    self._motors[id[0]][subid[0]][index[0]] = data[0]
            if func_code[0] == self._FUNC_WRITE_OK:
                if id[0] < 64:
                    self.link_status[id[0]] = 1
                if subid[0] > 0:
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
                if len(self._array_p_0) != 0:
                    # print(len(self._array_p_0))
                    self._connection.write(self.msg32)
                    rx = self._connection.read(32)
                    if rx[2] != 10:
                        struct.pack_into('i', self.msg32, 0, int(*(self._array_p_0.pop(0),)))
                        struct.pack_into('i', self.msg32, 4, int(*(self._array_p_1.pop(0),)))
                        struct.pack_into('i', self.msg32, 8, int(*(self._array_p_2.pop(0),)))
                        struct.pack_into('i', self.msg32, 12, int(*(self._array_p_3.pop(0),)))
                        struct.pack_into('i', self.msg32, 16, int(*(self._array_p_4.pop(0),)))
                        struct.pack_into('i', self.msg32, 20, int(*(self._array_p_5.pop(0),)))
                        struct.pack_into('i', self.msg32, 24, int(*(self._array_p_6.pop(0),)))
                        struct.pack_into('i', self.msg32, 28, int(*(self._array_p_7.pop(0),)))
                    self._connection.reset_input_buffer()
                else:
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
        print('Pyhton SDK for DBD BeeS Stopped')

    def tunePhase(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_TUNING, id, 0, 1)

    # set sync interpolation position
    def setSIPose(self, dt, pos):
        timeline = []
        pos_0 = []
        pos_1 = []
        pos_2 = []
        pos_3 = []
        pos_4 = []
        pos_5 = []
        pos_6 = []
        pos_7 = []
        timeline.append(0)
        pos_0.append(self._last_si_pos[0])
        pos_1.append(self._last_si_pos[1])
        pos_2.append(self._last_si_pos[2])
        pos_3.append(self._last_si_pos[3])
        pos_4.append(self._last_si_pos[4])
        pos_5.append(self._last_si_pos[5])
        pos_6.append(self._last_si_pos[6])
        pos_7.append(self._last_si_pos[7])

        timeline.append(1)
        pos_0.append(self._last_si_pos[0])
        pos_1.append(self._last_si_pos[1])
        pos_2.append(self._last_si_pos[2])
        pos_3.append(self._last_si_pos[3])
        pos_4.append(self._last_si_pos[4])
        pos_5.append(self._last_si_pos[5])
        pos_6.append(self._last_si_pos[6])
        pos_7.append(self._last_si_pos[7])

        timeline.append(dt - 1)
        pos_0.append(pos[0])
        pos_1.append(pos[1])
        pos_2.append(pos[2])
        pos_3.append(pos[3])
        pos_4.append(pos[4])
        pos_5.append(pos[5])
        pos_6.append(pos[6])
        pos_7.append(pos[7])

        timeline.append(dt)
        pos_0.append(pos[0])
        pos_1.append(pos[1])
        pos_2.append(pos[2])
        pos_3.append(pos[3])
        pos_4.append(pos[4])
        pos_5.append(pos[5])
        pos_6.append(pos[6])
        pos_7.append(pos[7])

        self._last_si_pos[0] = pos[0]
        self._last_si_pos[1] = pos[1]
        self._last_si_pos[2] = pos[2]
        self._last_si_pos[3] = pos[3]
        self._last_si_pos[4] = pos[4]
        self._last_si_pos[5] = pos[5]
        self._last_si_pos[6] = pos[6]
        self._last_si_pos[7] = pos[7]

        cs_0 = PchipInterpolator(timeline, pos_0)
        cs_1 = PchipInterpolator(timeline, pos_1)
        cs_2 = PchipInterpolator(timeline, pos_2)
        cs_3 = PchipInterpolator(timeline, pos_3)
        cs_4 = PchipInterpolator(timeline, pos_4)
        cs_5 = PchipInterpolator(timeline, pos_5)
        cs_6 = PchipInterpolator(timeline, pos_6)
        cs_7 = PchipInterpolator(timeline, pos_7)

        x = range(0, dt, 1)
        y_0 = cs_0(x)
        y_1 = cs_1(x)
        y_2 = cs_2(x)
        y_3 = cs_3(x)
        y_4 = cs_4(x)
        y_5 = cs_5(x)
        y_6 = cs_6(x)
        y_7 = cs_7(x)
        for i in range(0, y_0.shape[0]):
            self._array_p_0.append(y_0[i])
            self._array_p_1.append(y_1[i])
            self._array_p_2.append(y_2[i])
            self._array_p_3.append(y_3[i])
            self._array_p_4.append(y_4[i])
            self._array_p_5.append(y_5[i])
            self._array_p_6.append(y_6[i])
            self._array_p_7.append(y_7[i])

    def waitSIP(self):
        while len(self._array_p_0) != 0:
            time.sleep(0.5)

    def setPowerOn(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 1)

    def setPowerOnPro(self, id, limit_soft, limit_off, auto_recovery):
        value = 0x1
        if limit_soft:
            value |= 0x2
        if auto_recovery:
            value |= 0x80
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

    def setEStopDec(self, id, dec):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ESTOP_DEC, id, 0,
                          dec)

    def setEStopMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_ESTOP_PROFILE)

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

    def setPowerLimit(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_POWER_LIMIT, id, 0, value)

    def setLimitPositionP(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_LIMIT_POSITION_P, id, 0, value)

    def setLimitPositionN(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_LIMIT_POSITION_N, id, 0, value)

    def setHomingTriggerLevel(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_TRIGGER, id, 0, value)

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
        self.retransmitLimit = 0
        print('Searching Online Devices...')
        if len(self.online) > 0:
            self.online.clear()
        for i in range(0, 64):
            self._motors[i][0][self._INDEX_BOARD_TYPE] = 0
        for i in range(0, 64):
            if self.getDeviceType(i) != 0:
                self.online.append(i)
        print('Online Devices:', self.online)
        self.retransmitLimit = 3
        return self.online

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_MEMORY, id, 0, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_DEVICE_ID, id, 0, value)
        self._delay()
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName, baudrate=2000000):
        self._connection = 0
        self._array_p_0 = []
        self._array_p_1 = []
        self._array_p_2 = []
        self._array_p_3 = []
        self._array_p_4 = []
        self._array_p_5 = []
        self._array_p_6 = []
        self._array_p_7 = []
        self._last_si_pos = []
        self._sip_buf = []
        self._current_pos = [0, 0, 0, 0, 0, 0, 0, 0]
        self._last_si_pos = [0, 0, 0, 0, 0, 0, 0, 0]
        self.msg32 = ctypes.create_string_buffer(32)

        self._thread_stop_flag = 0
        self._tx_queue = queue.Queue()
        self._tx_lock = threading.Lock()
        self._rx_lock = threading.Lock()
        self._portName = portName
        self._baudrate = baudrate
        self._error_axis_num = -1
        array = (((ctypes.c_int32 * 64) * 8) * 64)
        self._motors = array()
        self.link_status = []
        self.actualPosition = []
        self.online = []
        for axis in range(0, 64):
            self.link_status.append(0)
            self.actualPosition.append(0)
        # print('Pyhton SDK for BeeS Started')
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()


class BeeAssistant:
    # Communication Profiles, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4
    _INDEX_IO_OUT = 23
    _INDEX_IO_OUT_ACC = 30
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
    _OPERATION_INDEX_MEMORY = 1
    _STATUS_DEVICE_ENABLE = 0X01
    _STATUS_HOMG_FIND = 0X02
    _STATUS_TARGET_REACHED = 0X04
    _STATUS_IO_INPUT = 0X08

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

            if func_code[0] == self._FUNC_READ_OK:
                self._motors[id[0]][subid[0]][index[0]] = data[0]

            self._rx_lock.release()

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    # def _linkProcess(self):
    #     while self._thread_stop_flag == 0:
    #         # while 1:
    #         self._tx_lock.acquire()
    #         # print('msg length:', self._tx_queue.qsize())
    #         if not self._tx_queue.empty():
    #             msg = self._tx_queue.get()
    #             self._tx_lock.release()
    #             self._connection.write(msg)
    #             self._analysis(self._connection.read(8))
    #             self._connection.reset_input_buffer()
    #         else:
    #             self._tx_lock.release()
    #             time.sleep(0.01)

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

    def print_hex(self, func, c_char_array):
        p_bytes = bytes(c_char_array)
        l = [hex(int(i)) for i in p_bytes]
        print(func, " ".join(l))

    def setPowerOn(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 1)
        self.print_hex("setPowerOn", self._tx_queue.get())

    def setPowerOff(self, id):
        # Disables power to the motor with the specified ID by sending a control word with value 0
        # Parameters:
        #   id: The ID of the motor to turn off
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)
        self.print_hex("setPowerOff", self._tx_queue.get())

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
        self.print_hex("setTargetVelocity", self._tx_queue.get())

    def setTargetPosition(self, id, value):
        # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
        self._motors[id][0][self._INDEX_TARGET_POSITION] = int(value)
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_POSITION, id, 0, value)
        self.print_hex("setTargetPosition", self._tx_queue.get())

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
            time.sleep(0.2)

    def getAccTime(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACC_TIME, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ACC_TIME]

    def setOutputIO(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_IO_OUT, id, 0, value)
        self.print_hex("setOutputIO", self._tx_queue.get())

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
        self.print_hex("getInputIO", self._tx_queue.get())
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

    def waitTargetPositionReached(self, id):
        condition = 1
        time.sleep(0.5)
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
    def __init__(self):
        self._connection = 0
        # Motors consist of 128 motor cells, each motor has 1024 parameters
        # self._motors = 0
        # self._thread1 = 0
        self._thread_stop_flag = 0
        self._tx_queue = queue.Queue()
        self._tx_lock = threading.Lock()
        self._rx_lock = threading.Lock()
        self._portName = '/dev/ttuUSB0'
        self._baudrate = 115200
        # self._connect()
        # self._thread1 = threading.Thread(target=self._linkProcess)
        # self._thread1.start()
        array = (((ctypes.c_int32 * 32) * 8) * 64)
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
        self._sendMessage(self._FUNC_WRITE, self._INDEX_DEVICE_ID, id, subid, value)
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
        # Disables power to the motor with the specified ID by sending a control word with value 0
        # Parameters:
        #   id: The ID of the motor to turn off
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
    _tx_pdo_queue = queue.Queue()
    _tx_lock = threading.Lock()
    _rx_lock = threading.Lock()
    msgPDO = ctypes.create_string_buffer(260)

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, self._baudrate, timeout=0.1)

    # Disconnect form the serial port and close the communication
    def _disconnect(self):
        self._thread_stop_flag = 1
        self._connection.close()

    def _delay(self):
        if self._baudrate == 2000000:
            time.sleep(0.05)
        else:
            time.sleep(0.1)

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
            # print('msg length:', self._tx_queue.qsize())
            if not self._tx_queue.empty():
                self._tx_lock.acquire()
                msg = self._tx_queue.get()
                self._tx_lock.release()
                self._connection.write(msg)
                self._analysis(self._connection.read(8))
                self._connection.reset_input_buffer()
            else:
                self._connection.write(self.msgPDO)
                time.sleep(0.05)
                self._connection.reset_input_buffer()

    def setPDO(self, n, h, m):
        struct.pack_into('B', self.msgPDO, 0, *(1,))
        struct.pack_into('B', self.msgPDO, 1, *(9,))
        struct.pack_into('i', self.msgPDO, 4 + n * 4, *(round(-m * 16384 * 50 / 12.0),))
        struct.pack_into('i', self.msgPDO, 4 + n * 4 + 32 * 4, *(round((h - 6) * 16384 * 50 / 12.0),))

    def setPara(self, index, value):
        struct.pack_into('B', self.msgPDO, 1, *(index,))
        for i in range(0, 64):
            struct.pack_into('i', self.msgPDO, 4 + i * 4, *(value,))

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
        self._delay()
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
        self._delay()
        return self._motors[id][subid][self._INDEX_ACC_TIME]

    def setOutputIO(self, id, subid, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_IO_OUT, id, subid, value)

    def getHomingLevel(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_LEVEL, id, subid, 0)
        self._delay()
        return self._motors[id][subid][self._INDEX_HOMING_LEVEL]

    def getHomingDirection(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_HOMING_DIRECTION, id, subid, 0)
        self._delay()
        return self._motors[id][subid][self._INDEX_HOMING_DIRECTION]

    def getKPP(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPP, id, subid, 0)
        self._delay()
        return self._motors[id][subid][self._INDEX_KPP]

    def getKPI(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPI, id, subid, 0)
        self._delay()
        return self._motors[id][subid][self._INDEX_KPI]

    def getKPD(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPD, id, subid, 0)
        self._delay()
        return self._motors[id][subid][self._INDEX_KPD]

    def getTargetCurrent(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_CURRENT, id, subid, 0)
        self._delay()
        return self._motors[id][subid][self._INDEX_TARGET_CURRENT]

    def getInputIO(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_IO_INPUT, id, subid, 0)
        self._delay()
        return self._motors[id][subid][self._INDEX_IO_INPUT]

    def getActualVelocity(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_VELOCITY, id, subid, 0)
        self._delay()
        return self._motors[id][subid][self._INDEX_ACTUAL_VELOCITY]

    def getActualPosition(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_POSITION, id, subid, 0)
        self._delay()
        return self._motors[id][subid][self._INDEX_ACTUAL_POSITION]

    def getTargetVelocity(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_VELOCITY, id, subid, 0)
        self._delay()
        return self._motors[id][subid][self._INDEX_TARGET_VELOCITY]

    def getTargetPosition(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_POSITION, id, subid, 0)
        self._delay()
        return self._motors[id][subid][self._INDEX_TARGET_POSITION]

    def getStatus(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_STATUS_WORD, id, subid, 0)
        self._delay()
        return self._motors[id][subid][self._INDEX_STATUS_WORD]

    def setEncoderPolarityP(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, subid, 1)

    def setEncoderPolarityN(self, id, subid):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, subid, -1)

    def getEncoderPolarity(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_POLARITY, id, subid, 0)
        self._delay()
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
        self._delay()
        return self._motors[id][subid][self._INDEX_DEVICE_ID]

    def getDeviceType(self, id, subid):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, subid, 0)
        self._delay()
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
        self._delay()
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName, baudrate=2000000):
        self._portName = portName
        self._baudrate = baudrate
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
    _INDEX_ACTUAL_CURRENT = 6
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

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, self._baudrate, timeout=0.1)

    # Disconnect form the serial port and close the communication
    def _disconnect(self):
        self._thread_stop_flag = 1
        self._connection.close()

    def _run(self):
        print("system run progress")

    def _delay(self):
        if self._baudrate == 2000000:
            time.sleep(0.05)
        else:
            time.sleep(0.1)

    # Analysis the Rx Message and Put the parameters into Each Motor
    # Note: need alarm message
    def _analysis(self, rx_message):
        if len(rx_message) == 8:
            self._rx_lock.acquire()
            func_code = struct.unpack_from('B', rx_message, 0)
            index = struct.unpack_from('B', rx_message, 1)
            id = struct.unpack_from('B', rx_message, 2)
            subid = struct.unpack_from('B', rx_message, 3)
            data = struct.unpack_from('i', rx_message, 4)

            if func_code[0] == self._FUNC_READ_OK:
                if id[0] < 32:
                    if subid[0] < 8:
                        if index[0] < 32:
                            self._motors[id[0]][subid[0]][index[0]] = data[0]

            self._rx_lock.release()
        # ret = 0
        # if len(rx_message) == 8:
        #     self._rx_lock.acquire()
        #     self.func_code = struct.unpack_from('B', rx_message, 0)
        #     self.index = struct.unpack_from('B', rx_message, 1)
        #     self.id = struct.unpack_from('B', rx_message, 2)
        #     self.subid = struct.unpack_from('B', rx_message, 3)
        #     self.data = struct.unpack_from('i', rx_message, 4)
        #
        #     if self.func_code[0] == self._FUNC_READ_OK:
        #         ret = 1
        #         self._motors[self.id[0]][self.subid[0]][self.index[0]] = self.data[0]
        #         if self.index[0] == self._INDEX_STATUS_WORD:
        #             if self.data[0] & self._STATUS_ERROR_OVERCURRENT == self._STATUS_ERROR_OVERCURRENT:
        #                 print("Error Over Current")
        #                 self._errorFlag = 1
        #                 # self.stop()
        #                 # exit(0)
        #             if self.data[0] & self._STATUS_ERROR_OVERPOSITION == self._STATUS_ERROR_OVERPOSITION:
        #                 print("Error Over Position")
        #                 self._errorFlag = 1
        #                 # self.stop()
        #                 # exit(0)
        #     if self.func_code[0] == self._FUNC_WRITE_OK:
        #         ret = 1
        #
        #     if self.func_code[0] == self._FUNC_FREE:
        #         ret = 1
        #
        #     if self.func_code[0] == self._FUNC_CHECK:
        #         self._sitp_buf_status = self.data[0]
        #         ret = 1
        #
        #     self._rx_lock.release()
        # else:
        #     # print("rx message error")
        #     # print(len(rx_message))
        #     ret = 0
        #
        # return ret

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        retransmitCounter = 0
        idleFlag = 0
        # msg = ctypes.create_string_buffer(8)
        while self._thread_stop_flag == 0:
            # if self._sitp_flag == 1:
            #     self._connection.write(self._private_msg)
            #     self._analysis(self._connection.read(8))
            #     self._connection.reset_input_buffer()
            #     while self._sitp_buf_status < 0x03:
            #         if not self._tx_sitp_queue.empty():
            #             self._sitp_msg = self._tx_sitp_queue.get()
            #         self._connection.write(self._sitp_msg)
            #         self._analysis(self._connection.read(8))
            #         self._connection.reset_input_buffer()
            # else:

            # if not self._tx_queue_pdo.empty():
            #     self._tx_lock_pdo.acquire()
            #     msg = self._tx_queue_pdo.get()
            #     self._tx_lock_pdo.release()
            #     self._connection.write(msg)
            #     self._analysis(self._connection.read(8))
            #     self._connection.reset_input_buffer()
            # else:
            #     if retransmitCounter == 0:
            if not self._tx_queue.empty():
                self._tx_lock.acquire()
                self._tx_message = self._tx_queue.get()
                self._tx_lock.release()
                self._connection.write(self._tx_message)
                self._analysis(self._connection.read(8))
                self._connection.reset_input_buffer()
                # idleFlag = 0
            else:
                time.sleep(0.01)
                # idleFlag = 1
                # else:
                #     idleFlag = 0
                # time.sleep(0.01)
                # if idleFlag == 1:
                #     time.sleep(0.01)
                # else:
                #     self._connection.write(self._tx_message)
                #     if (self._analysis(self._connection.read(8)) == 1):
                #         retransmitCounter = 0
                #     else:
                #         retransmitCounter = retransmitCounter + 1
                #         if retransmitCounter > self.retransmitLimit:
                #             retransmitCounter = 0
                #     self._connection.reset_input_buffer()

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
        # Disables power to the motor with the specified ID by sending a control word with value 0
        # Parameters:
        #   id: The ID of the motor to turn off
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)

    def setTargetVelocity(self, id, value) -> object:
        # Note: Unit: pulse/ms (51200 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 3000 is reasonable, higher speed will lose steps
        print("RPM:", value * 100 / 4096 * 60 / 8)
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, id, 0, value)

    def setTargetPosition(self, id, value):
        self._motors[id][0][self._INDEX_TARGET_POSITION] = int(value)
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
        self._delay()
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
        self._delay()
        return self._motors[id][0][self._INDEX_ACC_TIME]

    def getActualCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_CURRENT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ACTUAL_CURRENT]

    def _getAlpha(self, id):
        self._sendMessage(self._FUNC_READ, 30, id, 0, 0)
        self._delay()
        return self._motors[id][0][30]

    def getADC(self, channel):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_POSITION, 4, channel, 0)
        self._delay()
        return self._motors[4][channel][self._INDEX_ACTUAL_POSITION]

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

    def getKPP(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPP, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_KPP]

    def getKPI(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPI, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_KPI]

    def getKPD(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPD, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_KPD]

    def getKFF(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KFF, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_KFF]

    def getEncoderValue(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_VALUE, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ENCODER_VALUE]

    def getTargetCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_CURRENT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_TARGET_CURRENT]

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

    def setEncoderPolarityP(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, 0, 1)

    def setEncoderPolarityN(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, 0, -1)

    def getEncoderPolarity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_POLARITY, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ENCODER_POLARITY]

    def waitHomingDone(self, id):
        condition = 1
        while condition:
            if (self.getStatus(id) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0

    def waitTargetPositionReached(self, id):
        condition = 1
        # self._motors[id][0][self._INDEX_STATUS_WORD] = 0
        while condition:
            self.getActualPosition(id)
            if self._motors[id][0][self._INDEX_ACTUAL_POSITION] == self._motors[id][0][self._INDEX_TARGET_POSITION]:
                condition = 0
            if self._errorFlag == 1:
                condition = 0

    def getDeviceID(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_DEVICE_ID, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_DEVICE_ID]

    def getDeviceType(self, id):
        self._motors[id][0][self._INDEX_BOARD_TYPE] = 0
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def getPhaseCorrectCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_PHASE_CORRECT_CURRENT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_PHASE_CORRECT_CURRENT]

    def getEncoderOffset(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_OFFSET, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ENCODER_OFFSET]

    def getCurrentMax(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_CURRENT_MAX, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_CURRENT_MAX]

    def getPositionErrorMax(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_POSITION_ERROR_MAX, id, 0, 0)
        self._delay()
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
        # print(online)
        return online

    def tunePhase(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_TUNING, id, 0, 1)

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_MEMORY, id, 0, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_DEVICE_ID, id, 0, value)
        self._delay()
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName, baudrate=2000000):
        # Variables
        self._connection = 0
        # Motors consist of 128 motor cells, each motor has 1024 parameters

        self._sitp_msg = ctypes.create_string_buffer(12)

        self._thread_stop_flag = 0
        self._tx_queue = queue.Queue()
        self._tx_queue_pdo = queue.Queue()
        self._tx_sitp_queue = queue.Queue()  # sitp-> sync interpolation target position
        self._tx_lock = threading.Lock()
        self._rx_lock = threading.Lock()
        self._tx_lock_pdo = threading.Lock()
        self._rx_lock_pdo = threading.Lock()
        self._sitp_buf_status = 0
        self._sitp_flag = 0
        self._tx_message = ctypes.create_string_buffer(8)
        self.rxmsg = ctypes.create_string_buffer(8)
        self.tempC = 0
        self._private_msg = ctypes.create_string_buffer(8)
        self._empty_msg = ctypes.create_string_buffer(8)
        self.func_code = 0
        self.index = 0
        self.id = 0
        self.data = 0
        self._errorFlag = 0
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
        self._baudrate = baudrate
        self._array = (((ctypes.c_int32 * 32) * 8) * 32)
        self._motors = self._array()
        self._portName = portName
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        print('Pyhton SDK for DBD BLDC Servo Motor Started')


class Bull:
    # Communication Profiles for StepperServo, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4
    _INDEX_TARGET_CURRENT = 5
    _INDEX_ACTUAL_CURRENT = 6
    _INDEX_IO_OUT = 23
    _INDEX_KPP = 17
    _INDEX_KPI = 18
    _INDEX_KVF = 19
    _INDEX_KFF = 20
    _INDEX_PHASE_CORRECT_CURRENT = 21
    _INDEX_HOMING_TRIGGER = 13
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

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, self._baudrate, timeout=0.1)

    # Disconnect form the serial port and close the communication
    def _disconnect(self):
        self._thread_stop_flag = 1
        self._connection.close()

    def _run(self):
        print("system run progress")

    def _delay(self):
        if self._baudrate == 2000000:
            time.sleep(0.05)
        else:
            time.sleep(0.1)

    # Analysis the Rx Message and Put the parameters into Each Motor
    # Note: need alarm message
    def _analysis(self, rx_message):
        if len(rx_message) == 8:
            self._rx_lock.acquire()
            func_code = struct.unpack_from('B', rx_message, 0)
            index = struct.unpack_from('B', rx_message, 1)
            id = struct.unpack_from('B', rx_message, 2)
            subid = struct.unpack_from('B', rx_message, 3)
            data = struct.unpack_from('i', rx_message, 4)

            if func_code[0] == self._FUNC_READ_OK:
                if id[0] < 32:
                    if subid[0] < 8:
                        if index[0] < 32:
                            self._motors[id[0]][subid[0]][index[0]] = data[0]

            self._rx_lock.release()
        # ret = 0
        # if len(rx_message) == 8:
        #     self._rx_lock.acquire()
        #     self.func_code = struct.unpack_from('B', rx_message, 0)
        #     self.index = struct.unpack_from('B', rx_message, 1)
        #     self.id = struct.unpack_from('B', rx_message, 2)
        #     self.subid = struct.unpack_from('B', rx_message, 3)
        #     self.data = struct.unpack_from('i', rx_message, 4)
        #
        #     if self.func_code[0] == self._FUNC_READ_OK:
        #         ret = 1
        #         self._motors[self.id[0]][self.subid[0]][self.index[0]] = self.data[0]
        #         if self.index[0] == self._INDEX_STATUS_WORD:
        #             if self.data[0] & self._STATUS_ERROR_OVERCURRENT == self._STATUS_ERROR_OVERCURRENT:
        #                 print("Error Over Current")
        #                 self._errorFlag = 1
        #                 # self.stop()
        #                 # exit(0)
        #             if self.data[0] & self._STATUS_ERROR_OVERPOSITION == self._STATUS_ERROR_OVERPOSITION:
        #                 print("Error Over Position")
        #                 self._errorFlag = 1
        #                 # self.stop()
        #                 # exit(0)
        #     if self.func_code[0] == self._FUNC_WRITE_OK:
        #         ret = 1
        #
        #     if self.func_code[0] == self._FUNC_FREE:
        #         ret = 1
        #
        #     if self.func_code[0] == self._FUNC_CHECK:
        #         self._sitp_buf_status = self.data[0]
        #         ret = 1
        #
        #     self._rx_lock.release()
        # else:
        #     # print("rx message error")
        #     # print(len(rx_message))
        #     ret = 0
        #
        # return ret

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        retransmitCounter = 0
        idleFlag = 0
        # msg = ctypes.create_string_buffer(8)
        while self._thread_stop_flag == 0:
            # if self._sitp_flag == 1:
            #     self._connection.write(self._private_msg)
            #     self._analysis(self._connection.read(8))
            #     self._connection.reset_input_buffer()
            #     while self._sitp_buf_status < 0x03:
            #         if not self._tx_sitp_queue.empty():
            #             self._sitp_msg = self._tx_sitp_queue.get()
            #         self._connection.write(self._sitp_msg)
            #         self._analysis(self._connection.read(8))
            #         self._connection.reset_input_buffer()
            # else:

            # if not self._tx_queue_pdo.empty():
            #     self._tx_lock_pdo.acquire()
            #     msg = self._tx_queue_pdo.get()
            #     self._tx_lock_pdo.release()
            #     self._connection.write(msg)
            #     self._analysis(self._connection.read(8))
            #     self._connection.reset_input_buffer()
            # else:
            #     if retransmitCounter == 0:
            if not self._tx_queue.empty():
                self._tx_lock.acquire()
                self._tx_message = self._tx_queue.get()
                self._tx_lock.release()
                self._connection.write(self._tx_message)
                self._analysis(self._connection.read(8))
                self._connection.reset_input_buffer()
                # idleFlag = 0
            else:
                time.sleep(0.01)
                # idleFlag = 1
                # else:
                #     idleFlag = 0
                # time.sleep(0.01)
                # if idleFlag == 1:
                #     time.sleep(0.01)
                # else:
                #     self._connection.write(self._tx_message)
                #     if (self._analysis(self._connection.read(8)) == 1):
                #         retransmitCounter = 0
                #     else:
                #         retransmitCounter = retransmitCounter + 1
                #         if retransmitCounter > self.retransmitLimit:
                #             retransmitCounter = 0
                #     self._connection.reset_input_buffer()

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
        # Disables power to the motor with the specified ID by sending a control word with value 0
        # Parameters:
        #   id: The ID of the motor to turn off
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)

    def setTargetVelocity(self, id, value):
        # Note: Unit: pulse/ms (51200 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 3000 is reasonable, higher speed will lose steps
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, id, 0, value)

    def setTargetPosition(self, id, value):
        self._motors[id][0][self._INDEX_TARGET_POSITION] = int(value)
        # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_POSITION, id, 0, value)

    def setVelocityMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_VELOCITY)

    def setPositionMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_POSITION)

    def setHomingMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_HOMING)

    def setHomingTriggerLevel(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_HOMING_TRIGGER, id, 0, value)

    def setSyncInterpolationPositionMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0,
                          self._OPERATION_SYNC_INTERPOLATION_POSITION)

    def setKPP(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPP, id, 0, value)

    def setKPI(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KPI, id, 0, value)

    def setKVF(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_KVF, id, 0, value)

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
        self._delay()
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
        self._delay()
        return self._motors[id][0][self._INDEX_ACC_TIME]

    def getActualCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_CURRENT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ACTUAL_CURRENT]

    def _getAlpha(self, id):
        self._sendMessage(self._FUNC_READ, 30, id, 0, 0)
        self._delay()
        return self._motors[id][0][30]

    def getADC(self, channel):
        self._sendMessage(self._FUNC_READ, self._INDEX_ACTUAL_POSITION, 4, channel, 0)
        self._delay()
        return self._motors[4][channel][self._INDEX_ACTUAL_POSITION]

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

    def getKPP(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPP, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_KPP]

    def getKPI(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPI, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_KPI]

    def getKPD(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KPD, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_KPD]

    def getKFF(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_KFF, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_KFF]

    def getEncoderValue(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_VALUE, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ENCODER_VALUE]

    def getTargetCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_TARGET_CURRENT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_TARGET_CURRENT]

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

    def setEncoderPolarityP(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, 0, 1)

    def setEncoderPolarityN(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_ENCODER_POLARITY, id, 0, -1)

    def getEncoderPolarity(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_POLARITY, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ENCODER_POLARITY]

    def waitHomingDone(self, id):
        condition = 1
        while condition:
            if (self.getStatus(id) & self._STATUS_TARGET_REACHED) == self._STATUS_TARGET_REACHED:
                condition = 0

    def waitTargetPositionReached(self, id):
        condition = 1
        # self._motors[id][0][self._INDEX_STATUS_WORD] = 0
        while condition:
            self.getActualPosition(id)
            if self._motors[id][0][self._INDEX_ACTUAL_POSITION] == self._motors[id][0][self._INDEX_TARGET_POSITION]:
                condition = 0
            if self._errorFlag == 1:
                condition = 0

    def getDeviceID(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_DEVICE_ID, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_DEVICE_ID]

    def getDeviceType(self, id):
        self._motors[id][0][self._INDEX_BOARD_TYPE] = 0
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_BOARD_TYPE]

    def getPhaseCorrectCurrent(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_PHASE_CORRECT_CURRENT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_PHASE_CORRECT_CURRENT]

    def getEncoderOffset(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_ENCODER_OFFSET, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_ENCODER_OFFSET]

    def getCurrentMax(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_CURRENT_MAX, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_CURRENT_MAX]

    def getPositionErrorMax(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_POSITION_ERROR_MAX, id, 0, 0)
        self._delay()
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
        # print(online)
        return online

    def tunePhase(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_TUNING, id, 0, 1)

    def saveParameters(self, id):
        self._sendMessage(self._FUNC_OPERATION, self._OPERATION_INDEX_MEMORY, id, 0, 1)

    def changeID(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_DEVICE_ID, id, 0, value)
        self._delay()
        self.saveParameters(value)

    # Init Process
    def __init__(self, portName, baudrate=2000000):
        # Variables
        self._connection = 0
        # Motors consist of 128 motor cells, each motor has 1024 parameters

        self._sitp_msg = ctypes.create_string_buffer(12)

        self._thread_stop_flag = 0
        self._tx_queue = queue.Queue()
        self._tx_queue_pdo = queue.Queue()
        self._tx_sitp_queue = queue.Queue()  # sitp-> sync interpolation target position
        self._tx_lock = threading.Lock()
        self._rx_lock = threading.Lock()
        self._tx_lock_pdo = threading.Lock()
        self._rx_lock_pdo = threading.Lock()
        self._sitp_buf_status = 0
        self._sitp_flag = 0
        self._tx_message = ctypes.create_string_buffer(8)
        self.rxmsg = ctypes.create_string_buffer(8)
        self.tempC = 0
        self._private_msg = ctypes.create_string_buffer(8)
        self._empty_msg = ctypes.create_string_buffer(8)
        self.func_code = 0
        self.index = 0
        self.id = 0
        self.data = 0
        self._errorFlag = 0
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
        self._baudrate = baudrate
        self._array = (((ctypes.c_int32 * 32) * 8) * 32)
        self._motors = self._array()
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
    _INDEX_PROFILE_TIME = 21
    _INDEX_IO_OUT = 23
    _INDEX_IO_INPUT = 22
    _INDEX_PWM1 = 24
    _INDEX_PWM2 = 25
    _INDEX_PWM3 = 26
    _INDEX_PWM4 = 27
    _INDEX_PROFILE_PWM1 = 28
    _INDEX_PROFILE_PWM2 = 29
    _INDEX_PROFILE_PWM3 = 30
    _INDEX_PROFILE_PWM4 = 31
    _FUNC_WRITE = 1
    _FUNC_READ = 0
    _FUNC_WRITE_OK = 3
    _FUNC_READ_OK = 2
    _FUNC_OPERATION = 4
    _FUNC_OPERATION_OK = 5
    _FUNC_FREE = 255
    _OPERATION_MODE_PWM = 0
    _OPERATION_MODE_PROFILE_PWM = 2

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

    # _tx_lock = threading.Lock()
    # _rx_lock = threading.Lock()

    # Connect to the serial port and establish the communication
    def _connect(self):
        self._connection = serial.Serial(self._portName, self._baudrate, timeout=0.01)

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
        # Disables power to the motor with the specified ID by sending a control word with value 0
        # Parameters:
        #   id: The ID of the motor to turn off
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)

    def setPWMMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PWM)

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

    def setProfilePWMMode(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_OPERATION_MODE, id, 0, self._OPERATION_MODE_PROFILE_PWM)

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

    def setProfilePWM1(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_PROFILE_PWM1, id, 0, value)

    def setProfilePWM2(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_PROFILE_PWM2, id, 0, value)

    def setProfilePWM3(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_PROFILE_PWM3, id, 0, value)

    def setProfilePWM4(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_PROFILE_PWM4, id, 0, value)

    def setProfileTime(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_PROFILE_TIME, id, 0, value)

    def _delay(self):
        if self._baudrate == 2000000:
            time.sleep(0.05)
        else:
            time.sleep(0.2)

    def getBoardType(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        self._delay()
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
        self._delay()
        return self._motors[id][0][self._INDEX_TARGET_CURRENT]

    def getInputIO(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_IO_INPUT, id, 0, 0)
        self._delay()
        return self._motors[id][0][self._INDEX_IO_INPUT]

    def getStatus(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_STATUS_WORD, id, 0, 0)
        self._delay()
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
        self._delay()
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
    def __init__(self, portName, baudrate=2000000):
        # Variables
        self._connection = 0
        # Motors consist of 128 motor cells, each motor has 1024 parameters
        # _motors = numpy.zeros((32, 8, 32), dtype=numpy.int32)
        array = (((ctypes.c_int32 * 32) * 8) * 32)
        self._motors = array()
        self._thread1 = 0
        self._baudrate = baudrate
        self._thread_stop_flag = 0
        self._tx_queue = queue.Queue()
        self._portName = portName
        self._connect()
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        print('Pyhton SDK for DBD Bee LED Controller Started')


class BeeEncoder:
    # Communication Profiles for BeeEncoders, do not change them!
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
            # print("rx message error")
            ret = 0

        return ret

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        # time_last = 0
        while self._thread_stop_flag == 0:

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
                # time.sleep(0.01)

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
                        self._encoders[0] = value[0]
                        self._encoders[1] = value[1]
                        self._encoders[2] = value[2]
                        self._encoders[3] = value[3]
                        self._encoders[4] = value[4]
                        self._encoders[5] = value[5]
                        self._encoders[6] = value[6]
                        self._encoders[7] = value[7]

                        value = struct.unpack_from('8i', enc1, 0)
                        self._encoders[8] = value[0]
                        self._encoders[9] = value[1]
                        self._encoders[10] = value[2]
                        self._encoders[11] = value[3]
                        self._encoders[12] = value[4]
                        self._encoders[13] = value[5]
                        self._encoders[14] = value[6]
                        self._encoders[15] = value[7]

                        value = struct.unpack_from('8i', enc2, 0)
                        self._encoders[16] = value[0]
                        self._encoders[17] = value[1]
                        self._encoders[18] = value[2]
                        self._encoders[19] = value[3]
                        self._encoders[20] = value[4]
                        self._encoders[21] = value[5]
                        self._encoders[22] = value[6]
                        self._encoders[23] = value[7]

                        value = struct.unpack_from('8i', enc3, 0)
                        self._encoders[24] = value[0]
                        self._encoders[25] = value[1]
                        self._encoders[26] = value[2]
                        self._encoders[27] = value[3]
                        self._encoders[28] = value[4]
                        self._encoders[29] = value[5]
                        self._encoders[30] = value[6]
                        self._encoders[31] = value[7]

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
        self._delay()
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
        self._delay()
        return self._motors[id][0][self._INDEX_DEVICE_ID]

    def getDeviceType(self, id):
        self._sendMessage(self._FUNC_READ, self._INDEX_BOARD_TYPE, id, 0, 0)
        self._delay()
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
        # Disables power to the motor with the specified ID by sending a control word with value 0
        # Parameters:
        #   id: The ID of the motor to turn off
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)

    def setTargetVelocity(self, id, value):
        # Note: Unit: pulse/ms (51200 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 3000 is reasonable, higher speed will lose steps
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, id, 0, value)

    def setTargetPosition(self, id, value):
        self._motors[id][0][self._INDEX_TARGET_POSITION] = value
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
        self._delay()
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
        self._connection = serial.Serial(self._portName, 115200, timeout=1)  # 115200

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
    def _sendMessage(self, mid, rgb):
        message = ctypes.create_string_buffer(258)
        struct.pack_into('B', message, 0, 0xdd)
        struct.pack_into('B', message, 1, 0x55)
        struct.pack_into('B', message, 2, 0xEE)
        struct.pack_into('B', message, 3, 0x00)
        struct.pack_into('B', message, 4, 0x00)
        struct.pack_into('B', message, 5, 0x00)
        struct.pack_into('B', message, 6, *(mid,))
        struct.pack_into('B', message, 7, 0x00)
        struct.pack_into('B', message, 8, 0x99)
        struct.pack_into('B', message, 9, 0x01)
        struct.pack_into('B', message, 10, 0x00)
        struct.pack_into('B', message, 11, 0x00)
        struct.pack_into('B', message, 12, 0x00)
        struct.pack_into('B', message, 13, 3 * 80)
        struct.pack_into('B', message, 14, 0x00)
        struct.pack_into('B', message, 15, 0x01)
        # 1
        for i in range(0, 80 * 3):
            struct.pack_into('B', message, 16 + i, *(rgb[i],))

        struct.pack_into('B', message, 16 + 240, 0xAA)
        struct.pack_into('B', message, 17 + 240, 0xBB)
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


# # Class for basic communication
# # Each Stepper Group consist of 128 motors
# class AntUSB:
#     # Communication Profiles, do not change them!
#     _INDEX_CONTROL_WORD = 0
#     _INDEX_OPERATION_MODE = 1
#     _INDEX_IO_OUT = 14
#     _INDEX_MEMORY = 30
#     _INDEX_DEVICE_ID = 31
#     _INDEX_RUNNING_CURRENT = 40
#     _INDEX_KEEPING_CURRENT = 42
#     _INDEX_HOMING_DIRECTION = 45
#     _INDEX_HOMING_LEVEL = 46
#     _INDEX_ACC_TIME = 47
#     _INDEX_TARGET_VELOCITY = 48
#     _INDEX_TARGET_POSITION = 49
#     _INDEX_ACTUAL_VELOCITY = 52
#     _INDEX_ACTUAL_POSITION = 53
#     _INDEX_IO_INPUT = 54
#     _INDEX_TX_FEEDBACK = 96
#     _INDEX_SINGLE_DIRECTION = 97
#     _SUBINDEX_WRITE = 0
#     _SUBINDEX_READ = 1
#     _SUBINDEX_WRITE_OK = 2
#     _SUBINDEX_READ_OK = 3
#     _OPERATION_MODE_PROFILE_VELOCITY = 21
#     _OPERATION_MODE_PROFILE_POSITION = 31
#     _OPERATION_MODE_HOMING = 40
#     _FUNC_CODE_TSDO = 0X580
#     _FUNC_CODE_FREE = 0X780
#     _FUNC_CODE_SYNC = 0X080
#
#     # # Variables
#     # _connection = 0
#     # # Motors consist of 128 motor cells, each motor has 1024 parameters
#     # _motors = numpy.zeros((128, 1024), dtype=numpy.int32)
#     # _thread1 = 0
#     # _thread_stop_flag = 0
#     # _tx_queue = queue.Queue()
#     # _tx_lock = threading.Lock()
#     # _rx_lock = threading.Lock()
#
#     # Connect to the serial port and establish the communication
#     def _connect(self):
#         # backend = usb.backend.libusb1.get_backend(find_library=lambda x: "/usr/lib/libusb-1.0.so")
#         # backend = usb.backend.libusb1.get_backend(find_library=lambda x: "/system/lib/libusb-1.0.so")
#         # self._dev = usb.core.find(idVendor=0x1a86, idProduct=0x7523, backend=backend)
#         self._dev = usb.core.find(idVendor=0x1a86, idProduct=0x7523)
#         reattach = False
#         if self._dev.is_kernel_driver_active(0):
#             reattach = True
#             self._dev.detach_kernel_driver(0)
#         # print(dev)
#
#         endpoint_in = self._dev[0][(0, 0)][0]
#         endpoint_out = self._dev[0][(0, 0)][1]
#
#         self._a = endpoint_out.bEndpointAddress
#         self._b = endpoint_in.bEndpointAddress
#
#         self._dev.ctrl_transfer(64, 161, 0, 0, [])
#         ret = self._dev.ctrl_transfer(192, 95, 0, 0, 2)
#         # print(ret)
#         self._dev.ctrl_transfer(64, 154, 4882, 22682, [])
#         self._dev.ctrl_transfer(64, 154, 3884, 4, [])
#         ret = self._dev.ctrl_transfer(192, 149, 9496, 0, 2)
#         # print(ret)
#         self._dev.ctrl_transfer(64, 154, 10023, 0, [])
#         self._dev.ctrl_transfer(64, 164, 255, 0, [])
#
#         ret = self._dev.ctrl_transfer(64, 161, 50076, 64651, [])
#         # print(ret)
#
#     # Disconnect form the serial port and close the communication
#     def _disconnect(self):
#         self._thread_stop_flag = 1
#         # self._connection.close()
#
#     # Analysis the Rx Message and Put the parameters into Each Motor
#     def _analysis(self, rx_message, flag):
#         ret = 0
#         if len(rx_message) == 11:
#             self._rx_lock.acquire()
#             id = struct.unpack_from('h', rx_message, 0)
#             index = struct.unpack_from('h', rx_message, 2)
#             subindex = struct.unpack_from('h', rx_message, 4)
#             data = struct.unpack_from('i', rx_message, 6)
#             size = struct.unpack_from('B', rx_message, 10)
#             self._rx_lock.release()
#             func_code = id[0] & 0xff80
#             if func_code != self._FUNC_CODE_FREE:
#                 device_id = id[0] & 0x007f
#                 if device_id < 128:
#                     if index[0] > -1:
#                         if index[0] < 1024:
#                             if subindex[0] == self._SUBINDEX_READ_OK:
#                                 self._motors[device_id][index[0]] = data[0]
#                                 ret = 1
#                             elif subindex[0] == self._SUBINDEX_WRITE_OK:
#                                 ret = 1
#             else:
#                 if flag == 1:
#                     ret = 1
#         else:
#             print(len(rx_message))
#         return ret
#
#     # Establish the Low Level Communication Process
#     # This Process Runs in a Background Process
#     def _linkProcess(self):
#         freeMsgFlag = 0
#         while self._thread_stop_flag == 0:
#             self._tx_lock.acquire()
#             # print('msg length:', self._tx_queue.qsize())
#             if self._retransmitCounter == 0:
#                 if not self._tx_queue.empty():
#                     self._msg = self._tx_queue.get()
#                     freeMsgFlag = 0
#                 else:
#                     struct.pack_into('h', self._msg, 0, self._FUNC_CODE_FREE)
#                     struct.pack_into('h', self._msg, 2, *(0,))
#                     struct.pack_into('h', self._msg, 4, *(0,))
#                     struct.pack_into('i', self._msg, 6, *(0,))
#                     # self._msg = struct.pack('hhhi', self._FUNC_CODE_FREE, 0, 0, 0)
#                     freeMsgFlag = 1
#             self._tx_lock.release()
#             # self._msg = bytearray(10)
#             self._dev.write(self._a, self._msg, 100)
#             if self._analysis(self._dev.read(self._b, 11, 100), freeMsgFlag) == 0:
#                 self._retransmitCounter = 0
#             else:
#                 self._retransmitCounter = 0
#
#     # Send Message Function, Users would Call this Function to Send Messages
#     def _sendMessage(self, func_code, device_id, index, sub_index, data):
#         message = bytearray(10)
#         id = func_code + device_id
#         struct.pack_into('h', message, 0, *(id,))
#         struct.pack_into('h', message, 2, *(index,))
#         struct.pack_into('h', message, 4, *(sub_index,))
#         struct.pack_into('i', message, 6, *(data,))
#         # message = struct.pack('hhhi', func_code + device_id, index, sub_index, data)
#         self._tx_lock.acquire()
#         self._tx_queue.put(message)
#         self._tx_lock.release()
#
#     # Stop the communication
#     def stop(self):
#         time.sleep(0.5)
#         self._thread_stop_flag = 1
#         print('Python SDK for DBD Ant USB Stopped.')
#
#     def setSingleDirectionPositive(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_SINGLE_DIRECTION, self._SUBINDEX_WRITE, 1)
#
#     def setSingleDirectionNegative(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_SINGLE_DIRECTION, self._SUBINDEX_WRITE, -1)
#
#     def setSingleDirectionOff(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_SINGLE_DIRECTION, self._SUBINDEX_WRITE, 0)
#
#     def setTxFeedBackOn(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TX_FEEDBACK, self._SUBINDEX_WRITE, 1)
#
#     def setTxFeedBackOff(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TX_FEEDBACK, self._SUBINDEX_WRITE, 0)
#
#     def setPowerOn(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_CONTROL_WORD, self._SUBINDEX_WRITE, 1)
#
#     def setPowerOff(self, id):
#         # Disables power to the motor with the specified ID by sending a control word with value 0
#         # Parameters:
#         #   id: The ID of the motor to turn off
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_CONTROL_WORD, self._SUBINDEX_WRITE, 0)
#
#     def setTargetVelocity(self, id, value):
#         # Note: Unit: pulse/ms (50000 pulse per round), this unit nearly equals to RPM
#         #       and for stepper motors, 0 to 300 is reasonable, higher speed will lose steps
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_VELOCITY, self._SUBINDEX_WRITE, value)
#
#     def setTargetPosition(self, id, value):
#         # Note: Unit pulse, with 50000 pulse per round, and the value in should in range from -2^31 to 2^31
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_POSITION, self._SUBINDEX_WRITE, value)
#
#     def setVelocityMode(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
#                           self._OPERATION_MODE_PROFILE_VELOCITY)
#
#     def setPositionMode(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
#                           self._OPERATION_MODE_PROFILE_POSITION)
#
#     def setHomingMode(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_OPERATION_MODE, self._SUBINDEX_WRITE,
#                           self._OPERATION_MODE_HOMING)
#
#     def setHomingDirection(self, id, value):
#         if value == 1:
#             self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_DIRECTION, self._SUBINDEX_WRITE,
#                               1)
#         elif value == -1:
#             self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_DIRECTION, self._SUBINDEX_WRITE,
#                               -1)
#         else:
#             print("wrong value, please try 1 or -1.")
#
#     def setHomingLevel(self, id, value):
#         if value == 1:
#             self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_LEVEL, self._SUBINDEX_WRITE,
#                               1)
#         elif value == 0:
#             self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_LEVEL, self._SUBINDEX_WRITE,
#                               0)
#         else:
#             print("wrong value, please try 1 or 0.")
#
#     def setRunningCurrent(self, id, value):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_RUNNING_CURRENT, self._SUBINDEX_WRITE, value)
#
#     def setKeepingCurrent(self, id, value):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_KEEPING_CURRENT, self._SUBINDEX_WRITE, value)
#
#     def setAccTime(self, id, value):
#         # Note: acc time is a parameter for accelation and deaccelation progress, unit is ms, normally 200ms to 1000ms is reasonable
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_ACC_TIME, self._SUBINDEX_WRITE, value)
#
#     def setOutputIO(self, id, value):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_IO_OUT, self._SUBINDEX_WRITE, value)
#
#     def getInputIO(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_IO_INPUT, self._SUBINDEX_READ, 0)
#         time.sleep(0.05)
#         return self._motors[id][self._INDEX_IO_INPUT]
#
#     def getActualVelocity(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_ACTUAL_VELOCITY, self._SUBINDEX_READ, 0)
#         time.sleep(0.05)
#         return self._motors[id][self._INDEX_ACTUAL_VELOCITY]
#
#     def getActualPosition(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_ACTUAL_POSITION, self._SUBINDEX_READ, 0)
#         time.sleep(0.05)
#         return self._motors[id][self._INDEX_ACTUAL_POSITION]
#
#     def getTargetVelocity(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_VELOCITY, self._SUBINDEX_READ, 0)
#         time.sleep(0.05)
#         return self._motors[id][self._INDEX_TARGET_VELOCITY]
#
#     def getTargetPosition(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_TARGET_POSITION, self._SUBINDEX_READ, 0)
#         time.sleep(0.05)
#         return self._motors[id][self._INDEX_TARGET_POSITION]
#
#     def getRunningCurrent(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_RUNNING_CURRENT, self._SUBINDEX_READ, 0)
#         time.sleep(0.05)
#         return self._motors[id][self._INDEX_RUNNING_CURRENT]
#
#     def getKeepingCurrent(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_KEEPING_CURRENT, self._SUBINDEX_READ, 0)
#         time.sleep(0.05)
#         return self._motors[id][self._INDEX_KEEPING_CURRENT]
#
#     def getAccTime(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_ACC_TIME, self._SUBINDEX_READ, 0)
#         time.sleep(0.05)
#         return self._motors[id][self._INDEX_ACC_TIME]
#
#     def getHomingDirection(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_DIRECTION, self._SUBINDEX_READ, 0)
#         time.sleep(0.05)
#         return self._motors[id][self._INDEX_HOMING_DIRECTION]
#
#     def getHomingLevel(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_HOMING_LEVEL, self._SUBINDEX_READ, 0)
#         time.sleep(0.05)
#         return self._motors[id][self._INDEX_HOMING_LEVEL]
#
#     def waitHomingDone(self, id):
#         condition = 1
#         while condition:
#             vel = self.getActualVelocity(id)
#             if (vel == 0):
#                 condition = 0
#
#     def waitTargetPositionReached(self, id):
#         condition = 1
#         while condition:
#             time.sleep(0.05)
#             vel = self.getActualVelocity(id)
#             if (vel == 0):
#                 condition = 0
#
#     # return value: 1 - success, -1 - timeout
#     def waitTargetPositionReachedTimeout(self, id, timeout):
#         condition = 1
#         counter = timeout / 50
#         ret = 0
#         t = 0
#         while condition:
#             vel = self.getActualVelocity(id)
#             if vel == 0:
#                 condition = 0
#                 ret = 1
#             t = t + 1
#             if t > counter:
#                 condition = 0
#                 ret = -1
#         return ret
#
#     def getDeviceID(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_DEVICE_ID, self._SUBINDEX_READ, 0)
#         time.sleep(0.05)
#         return self._motors[id][self._INDEX_DEVICE_ID]
#
#     def scanDevices(self):
#         online = []
#         self._retransmitLimit = 1
#         print('Searching Online Devices...')
#         for i in range(0, 121):
#             self._motors[i][self._INDEX_DEVICE_ID] = 0
#         for i in range(1, 121):
#             if i == self.getDeviceID(i):
#                 online.append(i)
#         print('Online Devices:')
#         self._retransmitLimit = 3
#         print(online)
#
#     def saveParameters(self, id):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_MEMORY, self._SUBINDEX_WRITE, 1)
#
#     def changeID(self, id, value):
#         self._sendMessage(self._FUNC_CODE_TSDO, id, self._INDEX_DEVICE_ID, self._SUBINDEX_WRITE, value)
#         time.sleep(0.05)
#         self.saveParameters(value)
#
#     # Init Process
#     def __init__(self):
#         # Variables
#         self._connection = 0
#         # Motors consist of 128 motor cells, each motor has 1024 parameters
#         # self._motors = numpy.zeros((128, 1024), dtype=numpy.int32)
#         array = ((ctypes.c_int32 * 128) * 1024)
#         self._motors = array()
#         self._thread_stop_flag = 0
#         self._retransmitCounter = 0
#         self._retransmitLimit = 10
#         self._tx_queue = queue.Queue()
#         self._tx_lock = threading.Lock()
#         self._rx_lock = threading.Lock()
#         self._dev = 0
#         self._a = 0
#         self._b = 0
#         # self._msg = ctypes.create_string_buffer(10)
#         self._msg = bytearray(10)
#         self._connect()
#         self._thread1 = threading.Thread(target=self._linkProcess)
#         self._thread1.start()
#         print('Pyhton SDK for DBD Ant USB Started')

class Laser:
    # Communication Profiles for BLDCS, do not change them!
    _INDEX_BOARD_TYPE = 0
    _INDEX_DEVICE_ID = 1
    _INDEX_CONTROL_WORD = 2
    _INDEX_OPERATION_MODE = 3
    _INDEX_STATUS_WORD = 4
    _INDEX_F = 5
    _INDEX_D = 6
    _INDEX_START = 9
    _INDEX_T1 = 11
    _INDEX_D1 = 21

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

    def setF(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_F, id, 0, value)

    def setD(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_D, id, 0, value)

    def setTD(self, id, num, t, d):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_T1 + num - 1, id, 0, t)
        self._sendMessage(self._FUNC_WRITE, self._INDEX_D1 + num - 1, id, 0, d)

    def setTDStart(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_START, id, 0, 1)

    def setPowerOn(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 1)

    def setPowerOff(self, id):
        # Disables power to the motor with the specified ID by sending a control word with value 0
        # Parameters:
        #   id: The ID of the motor to turn off
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)

    def setTargetVelocity(self, id, value):
        # Note: Unit: pulse/ms (51200 pulse per round), this unit nearly equals to RPM
        #       and for stepper motors, 0 to 3000 is reasonable, higher speed will lose steps
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_VELOCITY, id, 0, value)

    def setTargetPosition(self, id, value):
        self._motors[id][0][self._INDEX_TARGET_POSITION] = value
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
        self._delay()
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
