#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2010-2011, Cody Jorgensen, Antons Rebguns.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of University of Arizona nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


__author__ = 'Cody Jorgensen, Antons Rebguns'
__copyright__ = 'Copyright (c) 2010-2011 Cody Jorgensen, Antons Rebguns'

__license__ = 'BSD'
__maintainer__ = 'Antons Rebguns'
__email__ = 'anton@email.arizona.edu'


import time
import serial
from array import array
from binascii import b2a_hex
from threading import Lock

from dynamixel_const import *

exception = None

class DynamixelIO(object):
    """ Provides low level IO with the Dynamixel servos through pyserial. Has the
    ability to write instruction packets, request and read register value
    packets, send and receive a response to a ping packet, and send a SYNC WRITE
    multi-servo instruction packet.
    """

    def __init__(self, port, baudrate, readback_echo=False):
        # Kinda cooking pasta now but since we are using 2 different motor types, this has to be done...
        self.motors_model = dict()

        """ Constructor takes serial port and baudrate as arguments. """
        try:
            self.serial_mutex = Lock()
            self.ser = None
            self.ser = serial.Serial(port, baudrate, timeout=0.015)
            self.port_name = port
            self.readback_echo = readback_echo
        except SerialOpenError:
           raise SerialOpenError(port, baudrate)

    def __del__(self):
        """ Destructor calls DynamixelIO.close """
        self.close()

    def close(self):
        """
        Be nice, close the serial port.
        """
        if self.ser:
            self.ser.flushInput()
            self.ser.flushOutput()
            self.ser.close()

    # Copied from pyxl320 package by Kevin Walchko, using the crazy DXL_CRC_TABLE
    # https://github.com/MomsFriendlyRobotCompany/pyxl320
    def __gen_crc16(self, data_blk):
        """
        Calculate crc

        in: data_blk - entire packet except last 2 crc bytes
        out: crc_accum - 16 word
        """
        data_blk_size = len(data_blk)
        crc_accum = 0
        for j in range(data_blk_size):
            i = ((crc_accum >> 8) ^ data_blk[j]) & 0xFF
            crc_accum = ((crc_accum << 8) ^ DXL_CRC_TABLE[i])
            crc_accum &= 0xffff  # keep to 16 bits

        return crc_accum

    def __write_serial(self, data):
        self.ser.flushInput()
        self.ser.flushOutput()
        self.ser.write(data)
        if self.readback_echo:
            self.ser.read(len(data))

    def __read_response(self, servo_id, protocol = 1):
        data = []
        
        # Obsolete protocol 1.0, keeping it here just in case we need it in the future
        if (protocol == 1):
            try:
                data.extend(self.ser.read(4))
                if not data[0:2] == ['\xff', '\xff']: raise Exception('Wrong packet prefix %s' % data[0:2])
                data.extend(self.ser.read(ord(data[3])))
                data = array('B', ''.join(data)).tolist() # [int(b2a_hex(byte), 16) for byte in data]
            except Exception, e:
                raise DroppedPacketError('Invalid response received from motor %d. %s' % (servo_id, e))
            checksum = 255 - sum(data[2:-1]) % 256
            if not checksum == data[-1]: raise ChecksumError(servo_id, data, checksum)

        # Protocol 2.0 to match with the later Dynamixel models
        elif (protocol == 2):

            # Not sure why there's a try exception here but whatever
            try:
                # Start with checking the package prefix first
                data.extend(self.ser.read(7))
                if data[0:4] != ['\xff', '\xff', '\xfd', '\x00']: raise Exception('Wrong packet prefix %s from motor %d' % (data[0:3], servo_id))

                # Prefix looks good, continue to fetch the rest of our data
                length = ord(data[5]) + (ord(data[6]) << 8)
                data.extend(self.ser.read(length))
                data = array('B', ''.join(data)).tolist() # I assumed this is to convert the packet to a list/array of something?

            except Exception, e:
                raise DroppedPacketError('Invalid response received from motor %d. %s' % (servo_id, e))

            # Okay looking good so far, let's check the checksum
            packet_checksum = data[-2] + (data[-1] << 8)
            if (packet_checksum != self.__gen_crc16(data[:-2])): raise ChecksumError(servo_id, data, packet_checksum)

        return data

    def read(self, servo_id, address, size, protocol = 1):
        """ Read "size" bytes of data from servo with "servo_id" starting at the
        register with "address". "address" is an integer between 0 and 57. It is
        recommended to use the constants in module dynamixel_const for readability.

        To read the position from servo with id 1, the method should be called
        like:
            read(1, DXL_GOAL_POSITION_L, 2)
        """
        # Obsolete protocol 1.0, keeping this here just in case
        if (protocol == 1):
            # Number of bytes following standard header (0xFF, 0xFF, id, length)
            length = 4  # instruction, address, size, checksum

            # directly from AX-12 manual:
            # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
            # If the calculated value is > 255, the lower byte is the check sum.
            checksum = 255 - ( (servo_id + length + DXL_READ_DATA + address + size) % 256 )

            # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
            packet = [0xFF, 0xFF, servo_id, length, DXL_READ_DATA, address, size, checksum]
            packetStr = array('B', packet).tostring() # same as: packetStr = ''.join([chr(byte) for byte in packet])

        # Protocol 2.0 for later Dynamixel compatible
        elif (protocol == 2):
            # Packet length
            length = 4 + 3 # Number of params (addr lower and higher + size lower and higher) + 3 (instruction + 2 checksum)

            # Building the packet
            # packet: (0xFF 0xFF 0xFD 0x00) ID (LEN_L LEN_H) INSTRUCTION (ADDR_L ADDR_H) (SIZE_L SIZE_H) (CRC_L CRC_H)
            packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, length&0xFF, (length>>8)&0xFF, DXL_READ_DATA, address&0xFF, (address>>8)&0xFF, size&0xFF, (size>>8)&0xFF]
            checksum = self.__gen_crc16(packet)
            packet.append(checksum&0xFF)
            packet.append((checksum>>8)&0xFF)

            # Okay convert to string for sending out
            packetStr = array('B', packet).tostring()

        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)#0.00235)

            # read response
            data = self.__read_response(servo_id, protocol)
            data.append(timestamp)

        return data

    def write(self, servo_id, address, data, protocol = 1):
        """ Write the values from the "data" list to the servo with "servo_id"
        starting with data[0] at "address", continuing through data[n-1] at
        "address" + (n-1), where n = len(data). "address" is an integer between
        0 and 49. It is recommended to use the constants in module dynamixel_const
        for readability. "data" is a list/tuple of integers.

        To set servo with id 1 to position 276, the method should be called
        like:
            write(1, DXL_GOAL_POSITION_L, (20, 1))
        """

        # Obsolete protocol 1.0, keeping this here just in case
        if (protocol == 1):
            # Number of bytes following standard header (0xFF, 0xFF, id, length)
            length = 3 + len(data)  # instruction, address, len(data), checksum

            # directly from AX-12 manual:
            # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
            # If the calculated value is > 255, the lower byte is the check sum.
            checksum = 255 - ((servo_id + length + DXL_WRITE_DATA + address + sum(data)) % 256)

            # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
            packet = [0xFF, 0xFF, servo_id, length, DXL_WRITE_DATA, address]
            packet.extend(data)
            packet.append(checksum)

        # Protocol 2.0 for later Dynamixel compatible
        elif (protocol == 2):
            # Packet length
            length = 5 + len(data) # instruction, 2byte address, len(data), 2byte CRC

            # Building the packet
            # packet: (0xFF 0xFF 0xFD 0x00) ID (LEN_L LEN_H) INSTRUCTION (ADDR_L ADDR_H) (DATA_0 DATA_1 DATA_2 DATA_3) (CRC_L CRC_H)
            packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, length&0xFF, (length>>8)&0xFF, DXL_WRITE_DATA, address&0xFF, (address>>8)&0xFF]
            packet.extend(data)

            # Calculating the checksum from the packet above
            checksum = self.__gen_crc16(packet)

            # # Very important note here: unlike the old checksum, protocol 2.0 checksum needs to
            # be calculated last, after everything has been put together
            packet.append(checksum&0xFF)
            packet.append((checksum>>8)&0xFF)

        # Convert to string and send out the data packet
        packetStr = array('B', packet).tostring() # packetStr = ''.join([chr(byte) for byte in packet])
        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)

            # read response
            data = self.__read_response(servo_id, protocol)
            data.append(timestamp)

        return data

    def sync_write(self, address, data, protocol = 1):
        """ Use Broadcast message to send multiple servos instructions at the
        same time. No "status packet" will be returned from any servos.
        "address" is an integer between 0 and 49. It is recommended to use the
        constants in module dynamixel_const for readability. "data" is a tuple of
        tuples. Each tuple in "data" must contain the servo id followed by the
        data that should be written from the starting address. The amount of
        data can be as long as needed.

        To set servo with id 1 to position 276 and servo with id 2 to position
        550, the method should be called like:
            sync_write(DXL_GOAL_POSITION_L, ( (1, 20, 1), (2 ,38, 2) ))
        """

        # Obsolete protocol 1.0, keeping this here just in case
        if (protocol == 1):
            # Calculate length and sum of all data
            flattened = [value for servo in data for value in servo]

            # Number of bytes following standard header (0xFF, 0xFF, id, length) plus data
            length = 4 + len(flattened)

            checksum = 255 - ((DXL_BROADCAST + length + \
                              DXL_SYNC_WRITE + address + len(data[0][1:]) + \
                              sum(flattened)) % 256)

            # packet: FF  FF  ID LENGTH INSTRUCTION PARAM_1 ... CHECKSUM
            packet = [0xFF, 0xFF, DXL_BROADCAST, length, DXL_SYNC_WRITE, address, len(data[0][1:])]
            packet.extend(flattened)
            packet.append(checksum)

        # Protocol 2.0 for later Dynamixel compatible
        elif (protocol == 2):
             # Calculate length and sum of all data
            flattened = [value for servo in data for value in servo]

            # Number of bytes following standard header (instruction, 2byte address, 2byte len, 2byte checksum) plus data
            length = 7 + len(flattened)

            # packet: FF FF FD 00 ID LEN_L LEN_H INSTRUCTION PARAM_1... CRC_L CRC_H
            packet = [0xFF, 0xFF, 0xFD, 0x00, DXL_BROADCAST, (length&0xFF), ((length>>8)&0xFF), DXL_SYNC_WRITE, (address&0xFF), ((address>>8)&0xFF), len(data[0][1:])&0xFF, (len(data[0][1:])>>8)&0xFF]
            packet.extend(flattened)

            # Very important note here: unlike the old checksum, protocol 2.0 checksum needs to
            # be calculated last, after everything has been put together
            checksum = self.__gen_crc16(packet)
            packet.append(checksum&0xFF)
            packet.append((checksum>>8)&0xFF)

        # Convert and send out message to motors
        packetStr = array('B', packet).tostring() # packetStr = ''.join([chr(byte) for byte in packet])
        with self.serial_mutex:
            self.__write_serial(packetStr)

    def ping(self, servo_id, protocol = 1):
        """ Ping the servo with "servo_id". This causes the servo to return a
        "status packet". This can tell us if the servo is attached and powered,
        and if so, if there are any errors.
        """

        # Below is the packet prepared for protocol 1.0
        if (protocol == 1):
            # Number of bytes following standard header (0xFF, 0xFF, id, length)
            length = 2  # instruction, checksum

            # directly from AX-12 manual:
            # Check Sum = ~ (ID + LENGTH + INSTRUCTION + PARAM_1 + ... + PARAM_N)
            # If the calculated value is > 255, the lower byte is the check sum.
            checksum = 255 - ((servo_id + length + DXL_PING) % 256)

            # packet: FF  FF  ID LENGTH INSTRUCTION CHECKSUM
            packet = [0xFF, 0xFF, servo_id, length, DXL_PING, checksum]
            packetStr = array('B', packet).tostring()

        # Then this is protol 2.0, refer to this website 
        # http://support.robotis.com/en/product/actuator/dynamixel_pro/communication/instruction_status_packet.htm
        # packet: (0xFF 0XFF 0XFD) (0X00) (ID) (LEN_L LEN_H) (Instruction) (Param1...ParamN) (CRC_L CRC_H)
        elif (protocol == 2):
            # Param + 3, but param = 0 for ping, thus self-explainatory
            length = 3
            
            # Build the packet here
            packet = [0xFF, 0xFF, 0xFD, 0x00, servo_id, (length & 0xFF), ((length >> 8) & 0xFF), DXL_PING]

            # Calculate the checksum and add it to the final packet
            checksum = self.__gen_crc16(packet)
            packet.append(checksum & 0xFF)
            packet.append((checksum >> 8) & 0xFF)

            # Convert to string
            packetStr = array('B', packet).tostring()

        with self.serial_mutex:
            self.__write_serial(packetStr)

            # wait for response packet from the motor
            timestamp = time.time()
            time.sleep(0.0013)

            # read response
            try:
                response = self.__read_response(servo_id, protocol)
                response.append(timestamp)
            except Exception, e:
                response = []

        if response:
            self.exception_on_error(response[4], servo_id, 'ping')

        return response

    def test_bit(self, number, offset):
        mask = 1 << offset
        return (number & mask)


    ######################################################################
    # These function modify EEPROM data which persists after power cycle #
    ######################################################################

    def set_id(self, old_id, new_id):
        """
        Sets a new unique number to identify a motor. The range from 1 to 253
        (0xFD) can be used.
        """
        response = self.write(old_id, DXL_ID, [new_id])
        if response:
            self.exception_on_error(response[4], old_id, 'setting id to %d' % new_id)
        return response

    def set_baud_rate(self, servo_id, baud_rate):
        """
        Sets servo communication speed. The range from 0 to 254.
        """
        response = self.write(servo_id, DXL_BAUD_RATE, [baud_rate])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting baud rate to %d' % baud_rate)
        return response

    def set_return_delay_time(self, servo_id, delay):
        """
        Sets the delay time from the transmission of Instruction Packet until
        the return of Status Packet. 0 to 254 (0xFE) can be used, and the delay
        time per data value is 2 usec.
        """
        response = self.write(servo_id, DXL_RETURN_DELAY_TIME, [delay])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting return delay time to %d' % delay)
        return response

    def set_angle_limit_cw(self, servo_id, angle_cw):
        """
        Set the min (CW) angle of rotation limit.
        """
        loVal = int(angle_cw % 256)
        hiVal = int(angle_cw >> 8)

        response = self.write(servo_id, DXL_CW_ANGLE_LIMIT_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CW angle limits to %d' % angle_cw)
        return response

    def set_angle_limit_ccw(self, servo_id, angle_ccw):
        """
        Set the max (CCW) angle of rotation limit.
        """
        loVal = int(angle_ccw % 256)
        hiVal = int(angle_ccw >> 8)

        response = self.write(servo_id, DXL_CCW_ANGLE_LIMIT_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CCW angle limits to %d' % angle_ccw)
        return response

    def set_angle_limits(self, servo_id, min_angle, max_angle):
        """
        Set the min (CW) and max (CCW) angle of rotation limits.
        """
        loMinVal = int(min_angle % 256)
        hiMinVal = int(min_angle >> 8)
        loMaxVal = int(max_angle % 256)
        hiMaxVal = int(max_angle >> 8)

        # set 4 register values with low and high bytes for min and max angles
        response = self.write(servo_id, DXL_CW_ANGLE_LIMIT_L, (loMinVal, hiMinVal, loMaxVal, hiMaxVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CW and CCW angle limits to %d and %d' %(min_angle, max_angle))
        return response

    def set_drive_mode(self, servo_id, is_slave=False, is_reverse=False):
        """
        Sets the drive mode for EX-106 motors
        """
        drive_mode = (is_slave << 1) + is_reverse

        response = self.write(servo_id, DXL_DRIVE_MODE, [drive_mode])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting drive mode to %d' % drive_mode)
        return response

    def set_voltage_limit_min(self, servo_id, min_voltage):
        """
        Set the minimum voltage limit.
        NOTE: the absolute min is 5v
        """

        if min_voltage < 5: min_voltage = 5
        minVal = int(min_voltage * 10)

        response = self.write(servo_id, DXL_DOWN_LIMIT_VOLTAGE, [minVal])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting minimum voltage level to %d' % min_voltage)
        return response

    def set_voltage_limit_max(self, servo_id, max_voltage):
        """
        Set the maximum voltage limit.
        NOTE: the absolute min is 25v
        """

        if max_voltage > 25: max_voltage = 25
        maxVal = int(max_voltage * 10)

        response = self.write(servo_id, DXL_UP_LIMIT_VOLTAGE, [maxVal])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting maximum voltage level to %d' % max_voltage)
        return response

    def set_voltage_limits(self, servo_id, min_voltage, max_voltage):
        """
        Set the min and max voltage limits.
        NOTE: the absolute min is 5v and the absolute max is 25v
        """

        if min_voltage < 5: min_voltage = 5
        if max_voltage > 25: max_voltage = 25

        minVal = int(min_voltage * 10)
        maxVal = int(max_voltage * 10)

        response = self.write(servo_id, DXL_DOWN_LIMIT_VOLTAGE, (minVal, maxVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting min and max voltage levels to %d and %d' %(min_voltage, max_voltage))
        return response


    ###############################################################
    # These functions can send a single command to a single servo #
    ###############################################################

    def set_torque_enabled(self, servo_id, enabled):
        """
        Sets the value of the torque enabled register to 1 or 0. When the
        torque is disabled the servo can be moved manually while the motor is
        still powered.
        """
        response = self.write(servo_id, DXL_TORQUE_ENABLE, [enabled])
        if response:
            self.exception_on_error(response[4], servo_id, '%sabling torque' % 'en' if enabled else 'dis')
        return response

    def set_compliance_margin_cw(self, servo_id, margin):
        """
        The error between goal position and present position in CW direction.
        The range of the value is 0 to 255, and the unit is the same as Goal Position.
        The greater the value, the more difference occurs.
        """
        response = self.write(servo_id, DXL_CW_COMPLIANCE_MARGIN, [margin])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CW compliance margin to %d' % margin)
        return response

    def set_compliance_margin_ccw(self, servo_id, margin):
        """
        The error between goal position and present position in CCW direction.
        The range of the value is 0 to 255, and the unit is the same as Goal Position.
        The greater the value, the more difference occurs.
        """
        response = self.write(servo_id, DXL_CCW_COMPLIANCE_MARGIN, [margin])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CCW compliance margin to %d' % margin)
        return response

    def set_compliance_margins(self, servo_id, margin_cw, margin_ccw):
        """
        The error between goal position and present position in CCW direction.
        The range of the value is 0 to 255, and the unit is the same as Goal Position.
        The greater the value, the more difference occurs.
        """
        response = self.write(servo_id, DXL_CW_COMPLIANCE_MARGIN, (margin_cw, margin_ccw))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CW and CCW compliance margins to values %d and %d' %(margin_cw, margin_ccw))
        return response

    def set_compliance_slope_cw(self, servo_id, slope):
        """
        Sets the level of Torque near the goal position in CW direction.
        Compliance Slope is set in 7 steps, the higher the value, the more flexibility is obtained.
        """
        response = self.write(servo_id, DXL_CW_COMPLIANCE_SLOPE, [slope])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CW compliance slope to %d' %  slope)
        return response

    def set_compliance_slope_ccw(self, servo_id, slope):
        """
        Sets the level of Torque near the goal position in CCW direction.
        Compliance Slope is set in 7 steps, the higher the value, the more flexibility is obtained.
        """
        response = self.write(servo_id, DXL_CCW_COMPLIANCE_SLOPE, [slope])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CCW compliance slope to %d' % slope)
        return response

    def set_compliance_slopes(self, servo_id, slope_cw, slope_ccw):
        """
        Sets the level of Torque near the goal position in CW/CCW direction.
        Compliance Slope is set in 7 steps, the higher the value, the more flexibility is obtained.
        """
        response = self.write(servo_id, DXL_CW_COMPLIANCE_SLOPE, (slope_cw, slope_ccw))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting CW and CCW compliance slopes to %d and %d' %(slope_cw, slope_ccw))
        return response

    def set_d_gain(self, servo_id, d_gain):
        """
        Sets the value of derivative action of PID controller.
        Gain value is in range 0 to 254.
        """
        response = self.write(servo_id, DXL_D_GAIN, [d_gain])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting D gain value of PID controller to %d' % d_gain)
        return response

    def set_i_gain(self, servo_id, i_gain):
        """
        Sets the value of integral action of PID controller.
        Gain value is in range 0 to 254.
        """
        response = self.write(servo_id, DXL_I_GAIN, [i_gain])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting I gain value of PID controller to %d' % i_gain)
        return response

    def set_p_gain(self, servo_id, p_gain):
        """
        Sets the value of proportional action of PID controller.
        Gain value is in range 0 to 254.
        """
        response = self.write(servo_id, DXL_P_GAIN, [p_gain])
        if response:
            self.exception_on_error(response[4], servo_id, 'setting P gain value of PID controller to %d' % p_gain)
        return response

    def set_punch(self, servo_id, punch):
        """
        Sets the limit value of torque being reduced when the output torque is
        decreased in the Compliance Slope area. In other words, it is the mimimum
        torque. The initial value is 32 (0x20) and can be extended up to 1023
        (0x3FF). (Refer to Compliance margin & Slope)
        """
        loVal = int(punch % 256)
        hiVal = int(punch >> 8)
        response = self.write(servo_id, DXL_PUNCH_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting punch to %d' % punch)
        return response

    def set_acceleration(self, servo_id, acceleration):
        """
        Sets the acceleration. The unit is 8.583 Degree / sec^2.
        0 - acceleration control disabled, 1-254 - valid range for acceleration.
        """

        model = self.get_model_number(servo_id)
        if not model in DXL_MODEL_TO_PARAMS:
            raise UnsupportedFeatureError(model, DXL_GOAL_ACCELERATION)

        if DXL_GOAL_ACCELERATION in DXL_MODEL_TO_PARAMS[model]['features']:
            response = self.write(servo_id, DXL_GOAL_ACCELERATION, (acceleration, ))
            if response:
                self.exception_on_error(response[4], servo_id, 'setting acceleration to %d' % acceleration)
            return response
        else:
            raise UnsupportedFeatureError(model, DXL_GOAL_ACCELERATION)

    def set_position(self, servo_id, position):
        """
        Set the servo with servo_id to the specified goal position.
        Position value must be positive.
        """
        protocol = self.motors_model[servo_id]['Protocol']
        loVal = int(position % 256)
        hiVal = int(position >> 8)

        response = self.write(servo_id, DXL_GOAL_POSITION_L, (loVal, hiVal), protocol)
        if response:
            self.exception_on_error(response[4], servo_id, 'setting goal position to %d' % position)
        return response

    def set_speed(self, servo_id, speed):
        """
        Set the servo with servo_id to the specified goal speed.
        Speed can be negative only if the dynamixel is in "freespin" mode.
        """

        # Packet to be sent out to the motors and motor type we are handling with
        protocol = self.motors_model[servo_id]['Protocol']

        # split speed into 2 bytes
        if speed >= 0:
            loVal = int(speed & 0xFF)
            hiVal = int(speed >> 8)
        else:
            loVal = int((1023 - speed) & 0xFF)
            hiVal = int((1023 - speed) >> 8)

        # set two register values with low and high byte for the speed
        response = self.write(servo_id, DXL_GOAL_SPEED_L, (loVal, hiVal), protocol)
        if response:
            self.exception_on_error(response[4], servo_id, 'setting moving speed to %d' % speed)
        return response

    def set_torque_limit(self, servo_id, torque):
        """
        Sets the value of the maximum torque limit for servo with id servo_id.
        Valid values are 0 to 1023 (0x3FF), and the unit is about 0.1%.
        For example, if the value is 512 only 50% of the maximum torque will be used.
        If the power is turned on, the value of Max Torque (Address 14, 15) is used as the initial value.
        """
        loVal = int(torque % 256)
        hiVal = int(torque >> 8)

        response = self.write(servo_id, DXL_TORQUE_LIMIT_L, (loVal, hiVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting torque limit to %d' % torque)
        return response

    def set_goal_torque(self, servo_id, torque):
        """
        Set the servo to torque control mode (similar to wheel mode, but controlling the torque)
        Valid values are from -1023 to 1023.
        Anything outside this range or 'None' disables torque control.
        """

        model = self.get_model_number(servo_id)
        if not model in DXL_MODEL_TO_PARAMS:
            raise UnsupportedFeatureError(model, DXL_TORQUE_CONTROL_MODE)

        valid_torque = torque is not None and torque >= -1023 and torque <= 1023
        if torque is not None and torque < 0:
            torque = 1024 - torque

        if DXL_TORQUE_CONTROL_MODE in DXL_MODEL_TO_PARAMS[model]['features']:
            if valid_torque:
                loVal = int(torque % 256); hiVal = int(torque >> 8);
                response = self.write(servo_id, DXL_GOAL_TORQUE_L, (loVal, hiVal))
                if response:
                    self.exception_on_error(response[4], servo_id, 'setting goal torque to %d' % torque)
            response = self.write(servo_id, DXL_TORQUE_CONTROL_MODE, (int(valid_torque), ))
            if response:
                self.exception_on_error(response[4], servo_id, 'enabling torque mode')
            return response
        else:
            raise UnsupportedFeatureError(model, DXL_TORQUE_CONTROL_MODE)

    def set_position_and_speed(self, servo_id, position, speed):
        """
        Set the servo with servo_id to specified position and speed.
        Speed can be negative only if the dynamixel is in "freespin" mode.
        """
        # split speed into 2 bytes
        if speed >= 0:
            loSpeedVal = int(speed % 256)
            hiSpeedVal = int(speed >> 8)
        else:
            loSpeedVal = int((1023 - speed) % 256)
            hiSpeedVal = int((1023 - speed) >> 8)

        # split position into 2 bytes
        loPositionVal = int(position % 256)
        hiPositionVal = int(position >> 8)

        response = self.write(servo_id, DXL_GOAL_POSITION_L, (loPositionVal, hiPositionVal, loSpeedVal, hiSpeedVal))
        if response:
            self.exception_on_error(response[4], servo_id, 'setting goal position to %d and moving speed to %d' %(position, speed))
        return response

    def set_led(self, servo_id, led_state):
        """
        Turn the LED of servo motor on/off.
        Possible boolean state values:
            True - turn the LED on,
            False - turn the LED off.
        """
        response = self.write(servo_id, DXL_LED, [led_state])
        if response:
            self.exception_on_error(response[4], servo_id,
                    'setting a LED to %s' % led_state)
        return response


    #################################################################
    # These functions can send multiple commands to multiple servos #
    # These commands are used in ROS wrapper as they don't send a   #
    # response packet, ROS wrapper gets motor states at a set rate  #
    #################################################################

    def set_multi_torque_enabled(self, valueTuples):
        """
        Method to set multiple servos torque enabled.
        Should be called as such:
        set_multi_servos_to_torque_enabled( (id1, True), (id2, True), (id3, True) )
        """
        # use sync write to broadcast multi servo message
        self.sync_write(DXL_TORQUE_ENABLE, tuple(valueTuples))

    def set_multi_compliance_margin_cw(self, valueTuples):
        """
        Set different CW compliance margin for multiple servos.
        Should be called as such:
        set_multi_compliance_margin_cw( ( (id1, margin1), (id2, margin2), (id3, margin3) ) )
        """
        self.sync_write(DXL_CW_COMPLIANCE_MARGIN, tuple(valueTuples))

    def set_multi_compliance_margin_ccw(self, valueTuples):
        """
        Set different CCW compliance margin for multiple servos.
        Should be called as such:
        set_multi_compliance_margin_ccw( ( (id1, margin1), (id2, margin2), (id3, margin3) ) )
        """
        self.sync_write(DXL_CCW_COMPLIANCE_MARGIN, tuple(valueTuples))

    def set_multi_compliance_margins(self, valueTuples):
        """
        Set different CW and CCW compliance margins for multiple servos.
        Should be called as such:
        set_multi_compliance_margins( ( (id1, cw_margin1, ccw_margin1), (id2, cw_margin2, ccw_margin2) ) )
        """
        self.sync_write(DXL_CW_COMPLIANCE_MARGIN, tuple(valueTuples))

    def set_multi_compliance_slope_cw(self, valueTuples):
        """
        Set different CW compliance slope for multiple servos.
        Should be called as such:
        set_multi_compliance_slope_cw( ( (id1, slope1), (id2, slope2), (id3, slope3) ) )
        """
        self.sync_write(DXL_CW_COMPLIANCE_SLOPE, tuple(valueTuples))

    def set_multi_compliance_slope_ccw(self, valueTuples):
        """
        Set different CCW compliance slope for multiple servos.
        Should be called as such:
        set_multi_compliance_slope_ccw( ( (id1, slope1), (id2, slope2), (id3, slope3) ) )
        """
        self.sync_write(DXL_CCW_COMPLIANCE_SLOPE, tuple(valueTuples))

    def set_multi_compliance_slopes(self, valueTuples):
        """
        Set different CW and CCW compliance slopes for multiple servos.
        Should be called as such:
        set_multi_compliance_slopes( ( (id1, cw_slope1, ccw_slope1), (id2, cw_slope2, ccw_slope2) ) )
        """
        self.sync_write(DXL_CW_COMPLIANCE_SLOPE, tuple(valueTuples))

    def set_multi_punch(self, valueTuples):
        """
        Set different punch values for multiple servos.
        NOTE: according to documentation, currently this value is not being used.
        Should be called as such:
        set_multi_punch( ( (id1, punch1), (id2, punch2), (id3, punch3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for sid,punch in valueTuples:
            # split punch into 2 bytes
            loVal = int(punch % 256)
            hiVal = int(punch >> 8)
            writeableVals.append( (sid, loVal, hiVal) )

        # use sync write to broadcast multi servo message
        self.sync_write(DXL_PUNCH_L, writeableVals)

    def set_multi_position(self, valueTuples):
        """
        Set different positions for multiple servos.
        Should be called as such:
        set_multi_position( ( (id1, position1), (id2, position2), (id3, position3) ) )
        """
        
        # Prepare different packets to send to each respective protocol
        pack_pro1 = []
        pack_pro2 = []

        # Iterate through our tuples and prepare some basic info first
        for vals in valueTuples:
            sid = vals[0]
            position = vals[1]
            protocol = self.motors_model[sid]['Protocol']
            dxl_model = self.motors_model[sid]['Model number']

            # split position into 2 bytes
            loVal = int(position % 256)
            hiVal = int(position >> 8)
            if (protocol == 1): pack_pro1.append((sid, loVal, hiVal))
            elif (protocol == 2): pack_pro2.append((sid, loVal, hiVal))

        # use sync write to broadcast multi servo message, send out each protocol packet accordingly
        if (pack_pro1): self.sync_write(DXL_GOAL_POSITION_L, pack_pro1, 1)
        if (pack_pro2): self.sync_write(DXL_GOAL_POSITION_L, pack_pro2, 2)
            
    def set_multi_speed(self, valueTuples):
        """
        Set different speeds for multiple servos.
        Should be called as such:
        set_multi_speed( ( (id1, speed1), (id2, speed2), (id3, speed3) ) )
        """
        # Prepare different packets to send to each respective protocol
        pack_pro1 = []
        pack_pro2 = []

        # Iterate through our tuples and prepare some basic info first
        for vals in valueTuples:
            sid = vals[0]
            speed = vals[1]
            protocol = self.motors_model[sid]['Protocol']
            dxl_model = self.motors_model[sid]['Model number']

            # split speed into 2 bytes
            if speed >= 0:
                loVal = int(speed % 256)
                hiVal = int(speed >> 8)
            else:
                loVal = int((1023 - speed) % 256)
                hiVal = int((1023 - speed) >> 8)

            if (protocol == 1): pack_pro1.append((sid, loVal, hiVal))
            elif (protocol == 2): pack_pro2.append((sid, loVal, hiVal))

        # use sync write to broadcast multi servo message
        if (pack_pro1): self.sync_write(DXL_GOAL_SPEED_L, pack_pro1, 1)
        if (pack_pro2): self.sync_write(DXL_GOAL_SPEED_L, pack_pro2, 2)

    def set_multi_torque_limit(self, valueTuples):
        """
        Set different torque limits for multiple servos.
        Should be called as such:
        set_multi_torque_limit( ( (id1, torque1), (id2, torque2), (id3, torque3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for sid,torque in valueTuples:
            # split torque into 2 bytes
            loVal = int(torque % 256)
            hiVal = int(torque >> 8)
            writeableVals.append( (sid, loVal, hiVal) )

        # use sync write to broadcast multi servo message
        self.sync_write(DXL_TORQUE_LIMIT_L, writeableVals)

    def set_multi_position_and_speed(self, valueTuples):
        """
        Set different positions and speeds for multiple servos.
        Should be called as such:
        set_multi_position_and_speed( ( (id1, position1, speed1), (id2, position2, speed2), (id3, position3, speed3) ) )
        """
        # prepare value tuples for call to syncwrite
        writeableVals = []

        for vals in valueTuples:
            sid = vals[0]
            position = vals[1]
            speed = vals[2]

            # split speed into 2 bytes
            if speed >= 0:
                loSpeedVal = int(speed % 256)
                hiSpeedVal = int(speed >> 8)
            else:
                loSpeedVal = int((1023 - speed) % 256)
                hiSpeedVal = int((1023 - speed) >> 8)

            # split position into 2 bytes
            loPositionVal = int(position % 256)
            hiPositionVal = int(position >> 8)
            writeableVals.append( (sid, loPositionVal, hiPositionVal, loSpeedVal, hiSpeedVal) )

        # use sync write to broadcast multi servo message
        self.sync_write(DXL_GOAL_POSITION_L, tuple(writeableVals))


    #################################
    # Servo status access functions #
    #################################

    def get_model_number(self, servo_id, protocol):
        """ Reads the servo's model number (e.g. 12 for AX-12+). """
        model = 0
        # Old method of getting model number from Protocol 1.0
        if (protocol == 1):
            response = self.read(servo_id, DXL_MODEL_NUMBER_L, 2, protocol)
            if response:
                self.exception_on_error(response[4], servo_id, 'fetching model number')
            model = response[5] + (response[6] << 8)

        # Model is embedded into ping return package in Protocol 2.0
        elif (protocol == 2):
            response = self.ping(servo_id, protocol)
            if response:
                self.exception_on_error(response[8], servo_id, 'fetching model number')
            model = response[9] + (response[10] << 8)

        # Got model, return here
        return model

    def get_firmware_version(self, servo_id, protocol=1):
        """ Reads the servo's firmware version. """
        response = self.read(servo_id, DXL_VERSION, 1, protocol)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching firmware version')
        if (protocol == 1): return response[5]
        elif (protocol == 2): return response[9]

    def get_return_delay_time(self, servo_id, protocol=1):
        """ Reads the servo's return delay time. """
        response = self.read(servo_id, DXL_RETURN_DELAY_TIME, 1, protocol)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching return delay time')
        if (protocol == 1): return response[5]
        elif (protocol == 2): return response[9]

    def get_angle_limits(self, servo_id, protocol = 1):
        """
        Returns the min and max angle limits from the specified servo.
        """
        # read in 4 consecutive bytes starting with low value of clockwise angle limit
        response = self.read(servo_id, DXL_CW_ANGLE_LIMIT_L, 4, protocol)
        cwLimit = 0.0
        ccwLimit = 0.0

        if response:
            self.exception_on_error(response[4], servo_id, 'fetching CW/CCW angle limits')
        
        # extract data valus from the raw data depending on the protocol used
        if (protocol == 1):
            cwLimit = response[5] + (response[6] << 8)
            ccwLimit = response[7] + (response[8] << 8)
        elif (protocol == 2):
            cwLimit = response[9] + (response[10] << 8)
            ccwLimit = response[11] + (response[12] << 8)

        # return the data in a dictionary
        return {'min':cwLimit, 'max':ccwLimit}

    def get_drive_mode(self, servo_id):
        """ Reads the servo's drive mode. """
        response = self.read(servo_id, DXL_DRIVE_MODE, 1)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching drive mode')
        return response[5]

    def get_voltage_limits(self, servo_id, protocol = 1):
        """
        Returns the min and max voltage limits from the specified servo.
        """
        # Empty response packet so we can request accordingly to motor models
        response = []
        min_voltage = 0.0
        max_voltage = 0.0

        # Again, we are having this thanks to the Dynamixel consistency
        motor_model = self.motors_model[servo_id]['Model number']
        if (motor_model == 29) or (motor_model == 12):
            response = self.read(servo_id, DXL_DOWN_LIMIT_VOLTAGE, 2, protocol)
        elif (motor_model == 350):
            response = self.read(servo_id, 13, 2, protocol)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching voltage limits')

        # extract data valus from the raw data, depending on different protocol
        if (protocol == 1):
            min_voltage = response[5] / 10.0
            max_voltage = response[6] / 10.0
        elif (protocol == 2):
            min_voltage = response[9] / 10.0
            max_voltage = response[10] / 10.0

        # return the data in a dictionary
        return {'min':min_voltage, 'max':max_voltage}

    def get_position(self, servo_id):
        """ Reads the servo's position value from its registers. """
        response = self.read(servo_id, DXL_PRESENT_POSITION_L, 2)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching present position')
        position = response[5] + (response[6] << 8)
        return position

    def get_speed(self, servo_id):
        """ Reads the servo's speed value from its registers. """
        response = self.read(servo_id, DXL_PRESENT_SPEED_L, 2)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching present speed')
        speed = response[5] + (response[6] << 8)
        if speed > 1023:
            return 1023 - speed
        return speed

    def get_voltage(self, servo_id, protocol = 1):
        """ Reads the servo's voltage. """
        # Empty response packet so we can request accordingly to motor models
        response = []

        # Consistency issues, who would have guessed
        motor_model = self.motors_model[servo_id]['Model number']
        if (motor_model == 29) or (motor_model == 12):
            response = self.read(servo_id, DXL_PRESENT_VOLTAGE, 1, protocol)
        elif (motor_model == 350):
            response = self.read(servo_id, 45, 1, protocol)
        if response:
            self.exception_on_error(response[4], servo_id, 'fetching supplied voltage')

        if (protocol == 1):
            return response[5] / 10.0
        elif (protocol == 2):
            return response[9] / 10.0
        else: self.exception_on_error("Incorrect protocol:", protocol)

    def get_current(self, servo_id):
        """ Reads the servo's current consumption (if supported by model) """
        model = self.get_model_number(servo_id)
        if not model in DXL_MODEL_TO_PARAMS:
            raise UnsupportedFeatureError(model, DXL_CURRENT_L)

        if DXL_CURRENT_L in DXL_MODEL_TO_PARAMS[model]['features']:
            response = self.read(servo_id, DXL_CURRENT_L, 2)
            if response:
                self.exception_on_error(response[4], servo_id, 'fetching sensed current')
            current = response[5] + (response[6] << 8)
            return 0.0045 * (current - 2048)

        if DXL_SENSED_CURRENT_L in DXL_MODEL_TO_PARAMS[model]['features']:
            response = self.read(servo_id, DXL_SENSED_CURRENT_L, 2)
            if response:
                self.exception_on_error(response[4], servo_id, 'fetching sensed current')
            current = response[5] + (response[6] << 8)
            return 0.01 * (current - 512)

        else:
            raise UnsupportedFeatureError(model, DXL_CURRENT_L)

    def get_feedback(self, servo_id, protocol):
        """
        Returns the id, goal, position, error, speed, load, voltage, temperature
        and moving values from the specified servo.
        """

        # Dictionary for return and motor model
        data_feedback = dict()
        dxl_model = self.motors_model[servo_id]['Model number']

        # Fetching feedback for the older MX-28 and AX-12
        if (dxl_model == 29) or (dxl_model == 12):
            # read in 17 consecutive bytes starting with low value for goal position
            response = self.read(servo_id, DXL_GOAL_POSITION_L, 17, protocol)

            if response:
                self.exception_on_error(response[4], servo_id, 'fetching full servo status')
            if len(response) == 24:
                # extract data values from the raw data
                goal = response[5] + (response[6] << 8)
                position = response[11] + (response[12] << 8)
                error = position - goal
                speed = response[13] + ( response[14] << 8)
                if speed > 1023: speed = 1023 - speed
                load_raw = response[15] + (response[16] << 8)
                load_direction = 1 if self.test_bit(load_raw, 10) else 0
                load = (load_raw & int('1111111111', 2)) / 1024.0
                if load_direction == 1: load = -load
                voltage = response[17] / 10.0
                temperature = response[18]
                moving = response[21]
                timestamp = response[-1]

                data_feedback = { 'timestamp': timestamp,
                 'id': servo_id,
                 'goal': goal,
                 'position': position,
                 'error': error,
                 'speed': speed,
                 'load': load,
                 'voltage': voltage,
                 'temperature': temperature,
                 'moving': bool(moving) }

        # Fetching feedback for the MX-28 firmware version 41
        elif (dxl_model == 30):
            # read in 20 consectutive bytes starting from 116 to 136
            response = self.read(servo_id, 116, 20, protocol)

            if response:
                self.exception_on_error(response[8], servo_id, 'fetching full servo status')
            if (len(response) == 32): # counting the additional timestamp as well
                # Start the data extraction process
                goal = response[9] + (response[10]<<8) + (response[11]<<16) + (response[12]<<24)
                position = response[25] + (response[26]<<8) + (response[27]<<16) + (response[28]<<24)
                error = position - goal

                # Done with the positions, moving over to speed
                speed = response[21] + (response[22]<<8) + (response[23]<<16) + (response[24]<<24)
                if speed > 1023: speed = (speed&0x3FF) - 1023

                # Now moving over to load
                load_raw = response[19] + (response[20]<<8)
                if (load_raw > 1000): load_raw -= 0xFFFF
                load = load_raw / 1000.0 # load somehow changed unit to 0.1%

                # Voltage, temperature not implemented...
                voltage = 0
                temperature = 0

                # Moving and timestamp
                moving = response[15]
                timestamp = response[-1]

                data_feedback = { 'timestamp': timestamp,
                 'id': servo_id,
                 'goal': goal,
                 'position': position,
                 'error': error,
                 'speed': speed,
                 'load': load,
                 'voltage': voltage,
                 'temperature': temperature,
                 'moving': bool(moving) }

        # Fetching feedback for the XL-320
        elif (dxl_model == 350):
            # read in 19 consecutive bytes starting from 30 to 49
            response = self.read(servo_id, 30, 19, protocol)

            if response:
                self.exception_on_error(response[8], servo_id, 'fetching full servo status')

            if (len(response) == 31): # counting the additional timestamp as well
                # Start the data extraction process
                goal = response[9] + (response[10]<<8)
                position = response[16] + (response[17]<<8)
                error = position - goal

                # Done with the positions, moving over to speed
                speed = response[18] + (response[19]<<8)
                if speed > 1023: speed = (speed&0x3FF) - 1023

                # Now moving over to load
                load_raw = response[20] + (response[21]<<8)
                if (load_raw > 1023): load_raw = 1023 - load_raw
                load = load_raw / 1024.0 # back to 0-1023 for this guy, great consistency Dynamixel

                # Voltage, temperature reading
                voltage = response[24] / 10.0
                temperature = response[25]

                # Moving and timestamp
                moving = response[28]
                timestamp = response[-1]

                data_feedback = { 'timestamp': timestamp,
                    'id': servo_id,
                    'goal': goal,
                    'position': position,
                    'error': error,
                    'speed': speed,
                    'load': load,
                    'voltage': voltage,
                    'temperature': temperature,
                    'moving': bool(moving) }

        # return the data in a dictionary
        return data_feedback

    def get_led(self, servo_id):
        """
        Get status of the LED. Boolean return values:
            True - LED is on,
            False - LED is off.
        """
        response = self.read(servo_id, DXL_LED, 1)
        if response:
            self.exception_on_error(response[4], servo_id,
                'fetching LED status')

        return bool(response[5])


    def exception_on_error(self, error_code, servo_id, command_failed):
        global exception
        exception = None
        ex_message = '[servo #%d on %s@%sbps]: %s failed' % (servo_id, self.ser.port, self.ser.baudrate, command_failed)

        if not isinstance(error_code, int):
            msg = 'Communcation Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, 0)
            return
        if not error_code & DXL_OVERHEATING_ERROR == 0:
            msg = 'Overheating Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & DXL_OVERLOAD_ERROR == 0:
            msg = 'Overload Error ' + ex_message
            exception = FatalErrorCodeError(msg, error_code)
        if not error_code & DXL_INPUT_VOLTAGE_ERROR == 0:
            msg = 'Input Voltage Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_ANGLE_LIMIT_ERROR == 0:
            msg = 'Angle Limit Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_RANGE_ERROR == 0:
            msg = 'Range Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_CHECKSUM_ERROR == 0:
            msg = 'Checksum Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)
        if not error_code & DXL_INSTRUCTION_ERROR == 0:
            msg = 'Instruction Error ' + ex_message
            exception = NonfatalErrorCodeError(msg, error_code)

    def import_motors_model(self, motors_model):
        self.motors_model = motors_model

class SerialOpenError(Exception):
    def __init__(self, port, baud):
        Exception.__init__(self)
        self.message = "Cannot open port '%s' at %d bps" %(port, baud)
        self.port = port
        self.baud = baud
    def __str__(self):
        return self.message

class ChecksumError(Exception):
    def __init__(self, servo_id, response, checksum):
        Exception.__init__(self)
        self.message = 'Checksum received from motor %d does not match the expected one (%d != %d)' \
                       %(servo_id, response[-1], checksum)
        self.response_data = response
        self.expected_checksum = checksum
    def __str__(self):
        return self.message

class FatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const
    def __str__(self):
        return self.message

class NonfatalErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const
    def __str__(self):
        return self.message

class ErrorCodeError(Exception):
    def __init__(self, message, ec_const):
        Exception.__init__(self)
        self.message = message
        self.error_code = ec_const
    def __str__(self):
        return self.message

class DroppedPacketError(Exception):
    def __init__(self, message):
        Exception.__init__(self)
        self.message = message
    def __str__(self):
        return self.message

class UnsupportedFeatureError(Exception):
    def __init__(self, model_id, feature_id):
        Exception.__init__(self)
        if model_id in DXL_MODEL_TO_PARAMS:
            model = DXL_MODEL_TO_PARAMS[model_id]['name']
        else:
            model = 'Unknown'
        self.message = "Feature %d not supported by model %d (%s)" %(feature_id, model_id, model)
    def __str__(self):
        return self.message

