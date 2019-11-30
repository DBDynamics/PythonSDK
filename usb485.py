import serial
import time
import struct
import ctypes
import threading
import queue
import numpy
import sys

# Parameters Dictionary-------------------------------------------------------------------------------------------------
# Index Dictionary
dictIndex = {
    # /*	Index	*/
    'DeviceIDIndex': 1,
    'IoOutIndex': 23,
    'OperationModeIndex': 3,
    'TargetPositionIndex': 9,
    'ActualPositionIndex': 10,
    'ProfileAccTimeIndex': 11,
    'TargetVelocityIndex': 7,
    'TargetCurrentIndex': 5,
    'ControlWordIndex': 2,
}

dictModes = {
    'OPMODE_HOMING': 40,
    'PROFILE_POSITION': 31,
}

# Function Code Dictionary
dictFuncCode = {
    # /* Function Code */
    'READ_SDO': 0,
    'WRITE_SDO': 1,
    'READ_SDO_OK': 2,
    'WRITE_SDO_OK': 3,
    'FREE': 255,
}


# End of Parameters Dictionary------------------------------------------------------------------------------------------

# Main Class -----------------------------------------------------------------------------------------------------------
# Each Stepper Group consist of 2048 motors
class DBDStepperGroup():
    connection = 0
    # Motors consist of 32*8 = 256 motor cells, each motor has 32 parameters
    motors = numpy.zeros((32, 8, 32), dtype=numpy.int32)
    thread1 = 0
    thread_stop_flag = 0
    tx_queue = queue.Queue()
    tx_lock = threading.Lock()
    rx_lock = threading.Lock()
    ioValue = 0

    # Connect to the serial port and establish the communication
    def connect(self):
        self.connection = serial.Serial(self.portName, 2000000, timeout=0.1)

    # Disconnect form the serial port and close the communication
    def disconnect(self):
        self.thread_stop_flag = 1
        self.connection.close()

    # Analysis the Rx Message and Put the parameters into Each Motor
    def analysis(self, rx_message):
        if len(rx_message) == 8:
            self.rx_lock.acquire()
            func_code = struct.unpack_from('B', rx_message, 0)
            index = struct.unpack_from('B', rx_message, 1)
            id = struct.unpack_from('B', rx_message, 2)
            subid = struct.unpack_from('B', rx_message, 3)
            data = struct.unpack_from('i', rx_message, 4)

            if func_code[0] == dictFuncCode['READ_SDO_OK']:
                self.motors[id[0]][subid[0]][index[0]] = data[0]

            self.rx_lock.release()

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def linkProcess(self):
        while self.thread_stop_flag == 0:
            # while 1:
            self.tx_lock.acquire()
            # print('msg length:', self.tx_queue.qsize())
            if not self.tx_queue.empty():
                msg = self.tx_queue.get()
            else:
                msg = ctypes.create_string_buffer(8)
                struct.pack_into('B', msg, 0, dictFuncCode['FREE'])
                struct.pack_into('B', msg, 1, *(0,))
                struct.pack_into('B', msg, 2, *(0,))
                struct.pack_into('B', msg, 3, *(0,))
                struct.pack_into('i', msg, 4, int(*(0,)))
            self.tx_lock.release()
            self.connection.write(msg)
            self.analysis(self.connection.read(8))

    # Send Message Function, Users would Call this Function to Send Messages
    def sendMessage(self, func_code, index, id, subid, data):
        # data = int(data)
        message = ctypes.create_string_buffer(8)
        struct.pack_into('B', message, 0, *(func_code,))
        struct.pack_into('B', message, 1, *(index,))
        struct.pack_into('B', message, 2, *(id,))
        struct.pack_into('B', message, 3, *(subid,))
        struct.pack_into('i', message, 4, int(*(data,)))
        self.tx_lock.acquire()
        self.tx_queue.put(message)
        # print('append new message')
        self.tx_lock.release()

    # Stop the communication
    def stop(self):
        time.sleep(1)
        self.thread_stop_flag = 1

    # Init Process
    def __init__(self, portName):
        self.portName = portName
        self.connect()
        self.thread1 = threading.Thread(target=self.linkProcess)
        self.thread1.start()

    # value 0 - disable, 1 - enable
    def setEnable(self, mid, msubid, value):
        self.sendMessage(dictFuncCode['WRITE_SDO'], dictIndex['ControlWordIndex'], mid, msubid, value)

    # value can be 100 to 3000 (mA) according to torque requirement
    def setCurent(self, mid, msubid, value):
        self.sendMessage(dictFuncCode['WRITE_SDO'], dictIndex['TargetCurrentIndex'], mid, msubid, value)

    def setSpeed(self, mid, msubid, value):
        self.sendMessage(dictFuncCode['WRITE_SDO'], dictIndex['TargetVelocityIndex'], mid, msubid, value)

    def setHoming(self, mid, msubid):
        self.sendMessage(dictFuncCode['WRITE_SDO'], dictIndex['OperationModeIndex'], mid, msubid,
                         dictModes['OPMODE_HOMING'])
    def waitHomingDone(self, mid, msubid):
        time.sleep(1)
        self.sendMessage(dictFuncCode['READ_SDO'], dictIndex['OperationModeIndex'], mid, msubid,0)
        time.sleep(1)
        while self.motors[mid][msubid][dictIndex['OperationModeIndex']] != dictModes['PROFILE_POSITION']:
            self.sendMessage(dictFuncCode['READ_SDO'], dictIndex['OperationModeIndex'], mid, msubid,
                         0)
            time.sleep(0.2)

    def setCurrent(self, mid, msubid, value):
        self.sendMessage(dictFuncCode['WRITE_SDO'], dictIndex['TargetCurrentIndex'], mid, msubid, value)

    def setPos(self, mid, msubid, value):
        self.motors[mid][msubid][dictIndex['TargetPositionIndex']] = value
        self.sendMessage(dictFuncCode['WRITE_SDO'], dictIndex['TargetPositionIndex'], mid, msubid, value)

    def waitPos(self, mid, msubid):
        while self.motors[mid][msubid][dictIndex['TargetPositionIndex']] != self.motors[mid][msubid][
            dictIndex['ActualPositionIndex']]:
            self.sendMessage(dictFuncCode['READ_SDO'], dictIndex['ActualPositionIndex'], mid, msubid, 0)
            time.sleep(0.1)

    # This Function Value will change All 22 IOs' State at One Time
    # 0 to 21 bit of this Value is mapped to IO0 to IO21
    def setIoOut(self, mid, msubid, value):
        self.sendMessage(dictFuncCode['WRITE_SDO'], dictIndex['IoOutIndex'], mid, msubid, value)

    # Value can be O or 1
    # Pin can be 0 to 21
    # Only change the selected pin's state, others keeping unchanged
    def setPinOut(self, mid, msubid, pin, value):
        if value == 0:
            self.ioValue &= ~ (0x1 << pin)
        elif value == 1:
            self.ioValue |= (0x1 << pin)

        self.setIoOut(mid, msubid, self.ioValue, )

    def demoTest(self):
        mid = 1
        msubid = 0
        for i in range(0, 3):
            self.setIoOut(mid, msubid, 0xffffff)
            time.sleep(0.2)
            self.setIoOut(mid, msubid, 0)
            time.sleep(0.2)

        for i in range(0, 22):
            self.setIoOut(mid, msubid, 0x1 << i)
            time.sleep(0.05)

        for i in range(0, 22):
            self.setIoOut(mid, msubid, 0x1 << (21 - i))
            time.sleep(0.05)

        for i in range(0, 3):
            self.setIoOut(mid, msubid, 0xffffff)
            time.sleep(0.2)
            self.setIoOut(mid, msubid, 0)
            time.sleep(0.2)

        self.setIoOut(mid, msubid, 0)

    def demoTest2(self):
        mid = 1
        msubid = 0

        for i in range(0, 22):
            self.setPinOut(mid, msubid, i, 1)
            time.sleep(0.05)

        for i in range(0, 22):
            self.setPinOut(mid, msubid, i, 0)
            time.sleep(0.05)

        for i in range(0, 22):
            self.setPinOut(mid, msubid, (21 - i), 1)
            time.sleep(0.05)

        for i in range(0, 22):
            self.setPinOut(mid, msubid, (21 - i), 0)
            time.sleep(0.05)
