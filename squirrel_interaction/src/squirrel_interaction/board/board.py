#!/usr/bin/env python

"""
    This Ros Node wraps the serial_api to communicate with the Interaction Board

    The name of the port to communicate with is retrieved from the "/squirrel_port" rosparam
    It defaults to /dev/ttyUSB0

    At a rate of 50Hz, it publishes motor positions to relevant topics, and actuates all devices

    List of topics:
        OUT
            /joint_states [JointState]
        IN
            /head_controller/joint_group_position_controller/command
            /neck_controller/joint_group_position_controller/command
            /camera_controller/joint_group_position_controller/command
            /light/command

    List of services:
            /door_controller/command
"""

import rospy
import serial_api
from serial import SerialException
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, ColorRGBA, String
from squirrel_interaction.srv import DoorController, StartMotor, StopMotor
from math import degrees, radians

class Controller:
    """Ros node to interface with interaction board """
    def __init__(self):
        self.head_destination = None
        self.neck_destination = None
        self.camera_destination = None
        self.base_lights = [0, 0, 0] * 42 # turns off all base leds
        self.mouth = [0, 0, 0] * 4
        self.door_open = False
        self.door_closed = False
        self.should_open_door = False
        self.should_close_door = False
        self._init_start_stop()
        rospy.init_node('squirrel_driver', anonymous=False)
        rospy.loginfo('Started squirrel_driver node')
        rospy.Subscriber('/head_controller/joint_group_position_controller/command', Float64, self._move_head)
        rospy.Subscriber('/neck_controller/joint_group_position_controller/command', Float64, self._move_neck)
        rospy.Subscriber('/camera_controller/joint_group_position_controller/command', Float64, self._move_camera)
        rospy.Subscriber('/light/command', ColorRGBA, self._change_base_light)
        rospy.Subscriber('/mouth/command', ColorRGBA, self._change_mouth)
        rospy.Service('door_controller/command', DoorController, self._move_door)
        rospy.Service('motor_controller/start', StartMotor, self._start_motor)
        rospy.Service('motor_controller/stop', StopMotor, self._stop_motor)
        self.position_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)

    def run(self):
        """ Port Writes """
        port = rospy.get_param("/squirrel_port")
        rate = rospy.Rate(50)
        with serial_api.Controller(port) as board:
            while not rospy.is_shutdown():
                for motor in self.start_stop_motors:
                    if self.start_stop_motors[motor] == -1:
                        board.stop_motor(motor)
                    if self.start_stop_motors[motor] == 1:
                        board.start_motor(motor)

                self._init_start_stop()
                # move motors
                if self.head_destination is not None:
                    board.move_to("head", self.head_destination)

                if self.neck_destination is not None:
                    board.move_to("neck", self.neck_destination)

                if self.camera_destination is not None:
                    board.move_to("camera", self.camera_destination)
                #get motor positions
                motors = ["head", "neck", "camera"]
                positions = board.get_positions()
                position = JointState(
                    name=motors,
                    position=positions)
                self.position_pub.publish(position)

                # write led colors
                board.set_base_led_colors(self.base_lights)
                board.set_mouth_led_colors(self.mouth)

                # get door position
                door = board.get_door_status()
                self.door_open = door == 'OPEN'
                self.door_closed = door == 'CLOSED'
		if self.door_open:
			self.should_open_door = False
		if self.door_closed:
			self.should_close_door = False
                # move door
                if self.should_open_door:
                    board.move_to('door', -30000)
                if self.should_close_door:
                    board.move_to('door', 30000)
                rate.sleep()

    def _move_head(self, message):
        self.head_destination = int(degrees(message.data))

    def _move_neck(self, message):
        self.neck_destination = int(degrees(message.data))

    def _move_camera(self, message):
        self.camera_destination = int(degrees(message.data))

    def _change_base_light(self, message):
        color = [int(message.r), int(message.g), int(message.b)]
        self.base_lights = color * 42   # number of base leds

    def _change_mouth(self, message):
        color = [int(message.r), int(message.g), int(message.b)]
        self.mouth = color * 4  # number of mouth leds

    def _move_door(self, req):
        if req.message == 'open':
            self.should_open_door = True
            self.should_close_door = False
            while not self.door_open:
                pass
            return True
        elif req.message == 'close':
            self.should_open_door = False
            self.should_close_door = True
            while not self.door_closed:
                pass
            return True
        return False

    def _start_motor(self, req):
        self.start_stop_motors[req.motor] = 1
        return True

    def _stop_motor(self, req):
        self.start_stop_motors[req.motor] = -1
        return True

    def _init_start_stop(self):
        self.start_stop_motors = {"head" : 0, "neck" : 0, "camera" : 0}

if __name__ == '__main__':
    try:
        Controller().run()
    except KeyError as ex:
        rospy.logerr("A rosparam was not correctly defined: /squirrel_port")
    except SerialException:
        rospy.logerr("Failed to contact interaction board. Verify /squirrel_port param")



