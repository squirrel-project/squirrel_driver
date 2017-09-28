#!/usr/bin/env python

""" The JoyController Node serves as an example client for the interaction components of SQUIRREL

    By subscribing to the /Joy [Joy] topic, this node allows the user to control:
        all motors of the interaction board.
        base leds,
        mouth leds,
        screen,
        speakers.

"""

from math import degrees, radians
from random import choice as random
import rospy
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64, String, ColorRGBA
from squirrel_interaction.srv import StartMotor, StopMotor, DisplayScreen, GetList, PlaySound, DoorController


class JoyController:
    def __init__(self):
        rospy.init_node('squirrel_joy', anonymous=False)
        self.head_command = rospy.Publisher('/head_controller/joint_group_position_controller/command', Float64, queue_size=10)
        self.neck_command = rospy.Publisher('/neck_controller/joint_group_position_controller/command', Float64, queue_size=10)
        self.camera_command = rospy.Publisher('/camera_controller/joint_group_position_controller/command', Float64, queue_size=10)
        self.face = rospy.ServiceProxy('face/emotion', DisplayScreen)
        self.door = rospy.ServiceProxy('door_controller/command', DoorController)
	self.base_lights = rospy.Publisher('/light/command', ColorRGBA, queue_size=10)
	self.head = Head(self.head_command)
        self.neck = Neck(self.neck_command)
        self.camera = Camera(self.camera_command)
        self._button_bindings = [
            self.leds_green,
            self.leds_red,
            self.leds_blue,
            self.expression,
            self.head.move_left,
            self.head.move_right,
            self.open_door,
            self.close_door,
            do_nothing,
            do_nothing,
            do_nothing
        ]
        self._axes_bindings = [
            do_nothing,
            do_nothing,
            do_nothing,
            do_nothing,
            do_nothing,
            do_nothing,
            self.neck.move,
            self.camera.move
        ]
        rospy.Subscriber('/joy', Joy, self.update_command)
	rospy.wait_for_service("/face/emotions")
        expressions = rospy.ServiceProxy("/face/emotions", GetList)
        self.expressions = expressions().list
	rospy.wait_for_service("/sound/sounds")
        sounds = rospy.ServiceProxy("/sound/sounds", GetList)
        self.sounds = sounds().list
	rospy.wait_for_service("/sound/play")
        self.play_sound = rospy.ServiceProxy("/sound/play", PlaySound)
 	self.move_mouth = rospy.Publisher("/mouth/speak", String, queue_size=10)
	self.expression(1)

    def update_command(self, message):
        for index in range(0, len(message.buttons)):
            self._button_bindings[index](message.buttons[index])
        for index in range(0, len(message.axes)):
            self._axes_bindings[index](message.axes[index])

    def look(self, image, pressed):
        if not pressed:
            return
        try:
            self.face(image)
        except rospy.ServiceException:
            rospy.logwarn('Failed to contact raspberry service.')

    def expression(self, pressed):
        if not pressed:
            return
        expression = random(self.expressions)
        self.look(expression, pressed)
        sound = random(self.sounds)
	self.move_mouth.publish(sound)
	self.play_sound(sound)


    def look_forward(self, pressed):
        self.look('look_front', pressed)

    def leds_green(self, pressed):
        if pressed:
            self.base_lights.publish(ColorRGBA(0, 255, 0, 0))

    def leds_red(self, pressed):
        if pressed:
            self.base_lights.publish(ColorRGBA(255, 0, 0, 0))

    def leds_blue(self, pressed):
        if pressed:
            self.base_lights.publish(ColorRGBA(0, 0, 255, 0))

    def open_door(self, pressed):
	if pressed:
	    self.door("open")
   
    def close_door(self, pressed):
	if pressed:
	    self.door("close")


def do_nothing(i):
    return


class Head:
    def __init__(self, pub):
        self._l = 0
        self._r = 0
        self.pub = pub
        self.start_motor = rospy.ServiceProxy('/motor_controller/start', StartMotor)
        self.stop_motor = rospy.ServiceProxy('/motor_controller/stop', StopMotor)

    def move_left(self, pressed):
        self._l = pressed

    # move_right is always called after move_left, so it can publish
    def move_right(self, pressed):
        self._r = pressed
        self._publish()

    def _publish(self):
        if self._l != self._r:
            self.pub.publish(radians(180) if self._r else radians(-180))
            self.start_motor("head")
        else:
            self.stop_motor("head")


class Neck(Head):
    def move(self, where):
        if not where:
            self.stop_motor("neck")
        else:
            self.pub.publish(radians(where * -1 * 180))
            self.start_motor("neck")

class Camera(Head):
    """ Camera tilt is limited between -68 and 30 degrees """
    def move(self, where):
        if not where:
            self.stop_motor("camera")
        else:
            destination = -67 if where < 0 else 29
            self.pub.publish(radians(destination))
            self.start_motor("camera")


if __name__ == '__main__':
    JoyController()
    rospy.spin()




