#!/usr/bin/python3

import ctypes
# import usb
# import usb.backend.libusb1
import math
import queue
import struct
import threading
import time

import serial
from scipy.interpolate import PchipInterpolator
from collections import deque

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
    _INDEX_ENCODER_VALUE = 26

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

    # Connect to the serial port and establish the communication
    def _connect(self):
        try:
            self._connection = serial.Serial(self._portName, self._baudrate, timeout=0.1)
        except serial.SerialException as e:
            print(f"Error opening serial port {self._portName}: {e}")
            raise

    # Disconnect form the serial port and close the communication
    def _disconnect(self):
        self._thread_stop_flag = 1
        if self._connection and self._connection.is_open:
            self._connection.close()

    # Analysis the Rx Message and Put the parameters into Each Motor
    def _analysis(self, rx_message):
        if len(rx_message) == 8:
            self._rx_lock.acquire()
            try:
                func_code = struct.unpack_from('B', rx_message, 0)
                index = struct.unpack_from('B', rx_message, 1)
                id = struct.unpack_from('B', rx_message, 2)
                subid = struct.unpack_from('B', rx_message, 3)
                data = struct.unpack_from('i', rx_message, 4)

                if func_code[0] == self._FUNC_READ_OK:
                    self._motors[id[0]][subid[0]][index[0]] = data[0]
            finally:
                self._rx_lock.release()

    # Establish the Low Level Communication Process
    # This Process Runs in a Background Process
    def _linkProcess(self):
        while self._thread_stop_flag == 0:
            self._tx_lock.acquire()
            if not self._tx_queue.empty():
                msg = self._tx_queue.get()
                self._tx_lock.release()
                if self._connection and self._connection.is_open:
                    self._connection.write(msg)
                    self._analysis(self._connection.read(8))
                    self._connection.reset_input_buffer()
            else:
                self._tx_lock.release()
                if self.syncInterpolationFlag == 1:
                    if self._connection and self._connection.is_open:
                        self._connection.write(self.msg32)
                        rx = self._connection.read(32)
                        if len(rx) >= 3 and rx[2] != 10:
                            for i in range(8):
                                if self._array_p[i]: # Check if deque is not empty
                                    val = int(self._array_p[i].popleft())
                                    struct.pack_into('i', self.msg32, i * 4, val)
                        self._connection.reset_input_buffer()
                else:
                    time.sleep(0.01)

    # wait SI Position Reached
    def waitSIP(self):
        while any(len(q) > 0 for q in self._array_p):
            time.sleep(0.5)

    # set sync interpolation position
    def setSIPose(self, dt, pos):
        timeline = [0, 1, dt - 1, dt]
        x = range(0, dt, 1)
        
        # Calculate interpolation for all 8 axes
        for i in range(8):
            p_vals = [
                self._last_si_pos[i],
                self._last_si_pos[i],
                pos[i],
                pos[i]
            ]
            cs = PchipInterpolator(timeline, p_vals)
            y = cs(x)
            self._array_p[i].extend(y)
            self._last_si_pos[i] = pos[i]

    def invK(self, x, y, z):
        k2 = 180.0 / math.pi
        l0 = math.sqrt(x * x + y * y + z * z)
        # Prevent math domain error
        arg = l0 / 2.0 / 120.0
        if arg > 1.0: arg = 1.0
        elif arg < -1.0: arg = -1.0
        theta = math.asin(arg) * k2
        
        alpha = 90.0 - theta
        
        arg_z = z / l0 if l0 != 0 else 0
        if arg_z > 1.0: arg_z = 1.0
        elif arg_z < -1.0: arg_z = -1.0
        
        if z >= 0:
            beta = math.asin(arg_z) * k2
            j1 = 90.0 - (alpha + beta)
        else:
            beta = math.asin(-arg_z) * k2
            j1 = 90.0 - (alpha - beta)

        j2 = 90.0 - 2 * theta + j1
        
        l1 = math.sqrt(x * x + y * y)
        arg_y = y / l1 if l1 != 0 else 0
        if arg_y > 1.0: arg_y = 1.0
        elif arg_y < -1.0: arg_y = -1.0

        if y >= 0:
            j0 = math.asin(arg_y) * k2
        else:
            j0 = -math.asin(-arg_y) * k2
        return j0, j1, j2

    def setLastSIPose(self, pos):
        for i in range(8):
            self._last_si_pos[i] = pos[i]

    def setSIPoseInvK(self, dt, pos):
        timeline = [0, 1, dt - 1, dt]
        x = range(0, dt, 1)
        y_arrays = []
        
        # Interpolate for all axes first
        for i in range(8):
            p_vals = [
                self._last_si_pos[i],
                self._last_si_pos[i],
                pos[i],
                pos[i]
            ]
            cs = PchipInterpolator(timeline, p_vals)
            y_arrays.append(cs(x))
            self._last_si_pos[i] = pos[i]

        k = 51200.0 * 90.0 / 20.0 / 360.0
        offset_j0 = -128
        offset_j1 = 50
        offset_j2 = 31
        
        # Apply Inverse Kinematics for first 3 axes and append all
        if y_arrays:
            for i in range(len(y_arrays[0])):
                # Inverse Kinematics for 0, 1, 2
                j0, j1, j2 = self.invK(y_arrays[0][i], y_arrays[1][i], y_arrays[2][i])
                self._array_p[0].append((-j0 - offset_j0) * k)
                self._array_p[1].append((-j1 - offset_j1) * k)
                self._array_p[2].append(-(-j2 - offset_j2) * k)
                
                # Direct append for 3-7
                for axis in range(3, 8):
                    self._array_p[axis].append(y_arrays[axis][i])

    def setCircularArcInvK(self, dt, start_pos_xy, end_pos_xy, center_pos_xy, direction=None, kinematics_func=None):
        """
        High-level wrapper for circular arc interpolation.
        Updates internal state automatically.
        Pure 2D interpolation (x, y).
        If direction is None, it is determined by the shortest path (<= 180 degrees).
        """
        start_pos = [start_pos_xy[0], start_pos_xy[1]]
        end_pos = [end_pos_xy[0], end_pos_xy[1]]
        center_pos = [center_pos_xy[0], center_pos_xy[1]]
        
        # 1. Generate 2D path points
        points = self.generate_arc_trajectory(start_pos, end_pos, center_pos, dt, direction)
        
        # 2. Apply kinematics
        # If kinematics_func is provided, use it. Otherwise use default.
        
        # Fetch current Z for compatibility with 3D kinematics functions
        current_z = self._last_si_pos[2]
        
        # Augment 2D points with current Z for the 3D kinematics model
        points_3d = [[p[0], p[1], current_z] for p in points]

        motor_data = self.compute_motor_steps(points_3d, kinematics_func)
        
        # 3. Append to queues
        for axis, values in motor_data.items():
            self.append_to_queue(axis, values)
            
        # 4. Handle other axes (hold position)
        for axis in range(8):
            if axis not in motor_data:
                # Repeat last position for dt steps
                last_val = self._last_si_pos[axis]
                self.append_to_queue(axis, [last_val] * dt)

        # 5. Update last known positions
        self.setLastSIPose(self._get_last_queue_positions())

    def generate_arc_trajectory(self, start_pos, end_pos, center_pos, dt, direction=None):
        """
        Generates a list of 2D points [x, y] forming a circular arc.
        Defined by Start, End, and Center points.
        Uses PchipInterpolator for smooth angle interpolation.
        """
        x1, y1 = start_pos[0], start_pos[1]
        x2, y2 = end_pos[0], end_pos[1]
        cx, cy = center_pos[0], center_pos[1]
        
        # Calculate radius from start point to center
        radius = math.sqrt((x1 - cx)**2 + (y1 - cy)**2)
        
        # Calculate start and end angles
        start_angle = math.atan2(y1 - cy, x1 - cx)
        end_angle = math.atan2(y2 - cy, x2 - cx)
        
        # Calculate angle difference
        diff = end_angle - start_angle
        
        # Normalize diff to [-pi, pi]
        while diff <= -math.pi:
            diff += 2 * math.pi
        while diff > math.pi:
            diff -= 2 * math.pi
            
        # If direction is explicitly provided, enforce it
        if direction == 'ccw':
            if diff <= 0: diff += 2 * math.pi
        elif direction == 'cw':
            if diff >= 0: diff -= 2 * math.pi
        # Else (direction is None), use the shortest path (diff is already in [-pi, pi])
        
        # Use PchipInterpolator for 1D interpolation of angle
        # Construct boundary conditions for S-curve profile
        # t: [0, 1, dt-1, dt]
        # val: [0, 0, diff, diff]
        
        if dt < 2:
             times = [0, 1]
             values = [0, diff]
             t_eval = [0]
        else:
             times = [0, 1, dt-1, dt]
             values = [0, 0, diff, diff]
             t_eval = range(dt)
             
        cs = PchipInterpolator(times, values)
        theta_offsets = cs(t_eval) # Returns numpy array of offsets
        
        points = []
        for offset in theta_offsets:
            current_angle = start_angle + offset
            
            # Polar to Cartesian
            px = cx + radius * math.cos(current_angle)
            py = cy + radius * math.sin(current_angle)
            
            # Map back to [x, y]
            pt = [px, py]
            points.append(pt)
            
        return points

    def compute_motor_steps(self, points, kinematics_func=None):
        """
        Converts Cartesian points to motor steps using a kinematics function.
        Returns dict: {axis_index: [step_values...]}
        """
        if kinematics_func is None:
            kinematics_func = self._default_kinematics_model

        result = {}
        for pt in points:
            motor_values = kinematics_func(pt[0], pt[1], pt[2])
            for axis, val in motor_values.items():
                if axis not in result:
                    result[axis] = []
                result[axis].append(val)
        return result

    def _default_kinematics_model(self, x, y, z):
        """
        Default Inverse Kinematics for the DBD Bee robot.
        Returns dict of {axis: steps}
        """
        j0, j1, j2 = self.invK(x, y, z)
        k = 51200.0 * 90.0 / 20.0 / 360.0
        offset_j0, offset_j1, offset_j2 = -128, 50, 31
        
        return {
            0: (-j0 - offset_j0) * k,
            1: (-j1 - offset_j1) * k,
            2: -(-j2 - offset_j2) * k
        }

    def append_to_queue(self, axis_id, data):
        """
        Appends a list of values to the specified axis queue.
        """
        self._array_p[axis_id].extend(data)

    def _get_last_queue_positions(self):
        """Helper to get the last planned position for all axes"""
        pos = [0] * 8
        for i in range(8):
            if self._array_p[i]:
                pos[i] = self._array_p[i][-1]
            else:
                pos[i] = self._last_si_pos[i]
        return pos


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
        while not self._tx_queue.empty():
            time.sleep(0.5)
        while any(len(q) > 0 for q in self._array_p):
            time.sleep(0.5)

        time.sleep(1)
        self._thread_stop_flag = 1
        print('Python SDK for DBD Bee Stopped')

    def setPowerOn(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 1)

    def setPowerOff(self, id):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_CONTROL_WORD, id, 0, 0)

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

    def setTP0(self, id, value) -> object:
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TP0, id, 0, value)

    def setTP1(self, id, value):
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TP1, id, 0, value)

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
        self._motors[id][0][self._INDEX_TARGET_POSITION] = int(value)
        self._sendMessage(self._FUNC_WRITE, self._INDEX_TARGET_POSITION, id, 0, value)

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
        for i in range(0, 32):
            self._motors[i][0][self._INDEX_BOARD_TYPE] = 0
        for i in range(0, 32):
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
        self.syncInterpolationFlag = 0
        
        # Use deque for better performance
        self._array_p = [deque() for _ in range(8)]
        
        self._current_pos = [0, 0, 0, 0, 0, 0, 0, 0]
        self._last_si_pos = [0, 0, 0, 0, 0, 0, 0, 0]
        self.msg32 = ctypes.create_string_buffer(32)

        self._thread_stop_flag = 0
        self._tx_queue = queue.Queue()
        self._tx_lock = threading.Lock()
        self._rx_lock = threading.Lock()
        self._portName = portName
        self._baudrate = baudrate
        
        try:
            self._connect()
        except Exception as e:
            print(f"Connection failed: {e}")
            
        self._thread1 = threading.Thread(target=self._linkProcess)
        self._thread1.start()
        array = (((ctypes.c_int32 * 32) * 8) * 32)
        self._motors = array()
        print('Python SDK for DBD Bee Started')
