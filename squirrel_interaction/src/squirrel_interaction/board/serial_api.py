#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" This Module serves as an interface for the interaction board present in the SQUIRREL robot

The board controls the following devices:

    - Head Motor (pan) (-180º ~ 180º) - head_controller

    - Neck Motor (pan) (-180º ~ 180º) - neck_pan_controller

    - Camera Motor (tilt) (-68º ~ 30º) - neck_tilt_controller

    - Door Motor (pan) (-30000, 30000)

    - 84 base RGB Led array

    - 4 mouth RGB Led array

"""

import time
from serial import Serial, SerialException
import binascii
from threading import Lock
import rospy

_MOTORS = ["head", "neck", "camera", "door"]
_MOTOR_SPEEDS = [60, 70, 70, 110]
_NUMBER_OF_BASE_LEDS = 42
_NUMBER_OF_MOUTH_LEDS = 4


class Controller(object):
    """ Main interface for Serial communication with Squirrel Interaction board """
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self._mutex = Lock()
        self.serial = Serial(port, baudrate)
        self.start_base_led_colors(_NUMBER_OF_BASE_LEDS)
        self.start_motors()

    def __exit__(self, exc_type, exc_value, exc_traceback):
        time.sleep(1)
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()
        self.serial.close()

    def clean(self):
        self.serial.reset_input_buffer()
        self.serial.reset_output_buffer()

    def reset(self):
        """ Resets each motor to Origin """
        for each_motor in ["head", "neck", "camera"]:
            self.move_to(each_motor, 0)

    def move_to(self, motor, degrees):
        """ Moves motor [param 0] to position [param 1] degrees """
        degrees = _valid_destination(motor, degrees)
        self._mutex.acquire()
	response=bytearray()
        try:
            message = bytearray(2)
            # simple bitwise operations to split an integer into 2 bytes
            message[0] = (degrees >> 8) & 0xff
            message[1] = degrees & 0xff
            self.serial.reset_input_buffer()
            rospy.loginfo("sending movement command")
            self.serial.write([
                0x30,
                _MOTORS.index(motor),
                message[0],
                message[1]
            ])
	    response = self._check_response(5, 0x30)
            rospy.loginfo("movement command done")

        except ValueError:
            return 'Available motors: head, neck, camera, door;'
        except SerialException:
            rospy.loginfo('Unable to access serial port. Is it still in use?')
        finally:
            self._mutex.release()
            return response

    def start_motors(self):
        """ Sets initial speeds for all Motors """
	self.reset()
        for motor in _MOTORS:
            self.start_motor(motor)

    def stop_motor(self, motor):
        """ Stops motor [param 0] """
        response=bytearray()
        try:
	    self._mutex.acquire()
            self.serial.reset_input_buffer()
	    self.serial.write([0x31, _MOTORS.index(motor), 0, 0])
            response = self._check_response(5, 0x31)
        except ValueError:
            return 'Available motors: head, neck, camera, door;'
	finally:
	    self._mutex.release()
            return response

    def start_motor(self, motor):
        """ Allows motor [param 0] to move again. By default, motors can move """
	response=bytearray()
        try:
            index = _MOTORS.index(motor)
	    self._mutex.acquire()
            self.serial.reset_input_buffer()
	    self.serial.write([0x31, index, 0, _MOTOR_SPEEDS[index]])
            response = self._check_response(5, 0x31)
        except ValueError:
            return 'Available motors: head, neck, camera, door;'
	finally:
	    self._mutex.release()
            return response

    def get_position(self, motor):
        """ Get motor [param 0] position in degrees """
        self._mutex.acquire()
	response=bytearray(4)
        try:
            self.serial.reset_input_buffer()
	    self.serial.write([
                0x52,
                _MOTORS.index(motor)
            ])
            response = bytearray(self.serial.read(7))
        except ValueError:
            return 'Available motors: head, neck, camera;'
        except SerialException:
            rospy.loginfo('Unable to access serial port. Is it still in use?')
        finally:
            self._mutex.release()
            return - (response[2] - response[3])
    def get_positions(self):
        """ Fetches all motor positions """
        return [self.get_position("head"), self.get_position("neck"), self.get_position("camera")]

    def set_mouth_led_colors(self, colors):
        """ Applies the specified RGBA value to all 4 mouth leds """
        if len(colors) != _NUMBER_OF_MOUTH_LEDS * 3:
            return 'Need RGB values (0-255, 0-255, 0-255) for each Base LED (%d)' % _NUMBER_OF_MOUTH_LEDS
        colors_ = map(_limit_color, colors)
        self._mutex.acquire()
        try:
            self.serial.reset_input_buffer()
            self.serial.write([0x3A] + colors_)
	    response = self._check_response(4, 0x3A)
        except SerialException:
            rospy.loginfo('Unable to access serial port. Is it still in use?')
        finally:
            self._mutex.release()
            return response

    def start_base_led_colors(self, number_of_leds):
        self._mutex.acquire()
	response = bytearray()
        try:
            self.serial.write([0x34, number_of_leds])
	    response = self._check_response(4, 0x34)
        except SerialException:
            rospy.loginfo('Unable to access serial port. Is it still in use?')
        finally:
            self._mutex.release()
            return response

    def set_base_led_colors(self, colors):
        """sets the rgb value for each base led (42 max)"""
	response = []
        if len(colors) != _NUMBER_OF_BASE_LEDS * 3:
            return 'Need RGB values (0-255, 0-255, 0-255) for each Base LED (%d)' % _NUMBER_OF_BASE_LEDS
        colors_ = map(_limit_color, colors)
        self._mutex.acquire()
        try:
            self.serial.reset_input_buffer()
            self.serial.write([0x35] + colors_)
            response = self._check_response(4, 0x35)
        except SerialException:
            rospy.loginfo('Unable to access serial port. Is it still in use?')
        finally:
            self._mutex.release()
            return response

    def get_door_status(self):
        self._mutex.acquire()
	response = []
        try:
            self.serial.reset_input_buffer()
            self.serial.write([
                0x55,
                3
            ])
            response = bytearray()
            response.extend(self.serial.read(6))
            if response[2] == 0x01:
                response = "OPEN"
            elif response[2] == 0x10:
                response = "CLOSED"
            else:
                response = "AJAR"
        except SerialException:
            rospy.loginfo('Unable to access serial port. Is it still in use?')
        finally:
            self._mutex.release()
            return response

    def _check_response(self, number_of_bytes_to_read, should_be):
        response = bytearray()
        try:
            response.extend(self.serial.read(number_of_bytes_to_read))
            return response[0] == should_be
        except SerialException:
            rospy.loginfo('Unable to access serial port. Is it still in use?')


def _limit_color(color_value):
    return min(max(color_value, 0), 255)


def _valid_destination(motor, degrees):
    if motor in ['head', 'neck']:
        return max(-50, min(50, degrees))
    if motor == 'camera':
        return max(-68, min(30, degrees))
    if motor == 'door':
        return max(-30000, min(30000, degrees))
    return False

