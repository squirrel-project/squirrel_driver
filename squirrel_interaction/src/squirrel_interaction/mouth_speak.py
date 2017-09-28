#!/usr/bin/env python
""" This Node controls Blinks the mouth leds when squirrel speaks

   It proves a service that should be called at the same time a sound clip is played.

   The argument given to the service is the sound file played.

   This None associates each available sound in $(find squirrel_interaction)/resources/sounds
   with an animation, produced dynamically on initialization

"""

import os
import wave
import random
import rospy
import rospkg
from std_msgs.msg import ColorRGBA, String

ROSPACK = rospkg.RosPack()
PATH_TO_SOUNDFILES = ROSPACK.get_path('squirrel_interaction') + "/resources/sounds/"
FRAMERATE = 30
COLORS = [[255, 255, 255, 0], [100, 255, 100, 0]]
DEFAULT_COLOR = [0, 0, 255, 0]


def is_sound_file(file_name):
    """ predicate """
    return file_name.endswith(".wav")


class Mouth(object):
    def __init__(self):
        """ Provides a Service that blinks the mouth leds

            The duration corresponds to how long the sound file is

        """
        rospy.init_node("mouth_animation", anonymous=False)
	rospy.Subscriber("/mouth/speak", String, self.__speak)
        self.pub = rospy.Publisher("/mouth/command", ColorRGBA, queue_size=10)
        self.frame_iterator = iter([])
        self.animations = self.__initialize_animations(PATH_TO_SOUNDFILES)

    def __speak(self, msg):
        try:
            filename = msg.data + ".wav"
            self.frame_iterator = iter(self.animations[filename])
            return True
        except KeyError:
            return False

    def run(self):
        """ Publishes a color, if robot is still speaking """
        try:
            color = self.frame_iterator.next()
            self.pub.publish(ColorRGBA(color[0], color[1], color[2], color[3]))
        except StopIteration:
            pass


    def __initialize_animations(self, path):
        animations = {}
        files = self.__get_file_names(path)
        for _f in files:
            animations[_f] = self.__produce_animation(path + _f)
            animations[_f].append(DEFAULT_COLOR)
        return animations


    def __get_file_names(self, path):
        return filter(is_sound_file, os.listdir(path))


    def __produce_animation(self, file_name):
        try:
            sound_file = wave.open(file_name, 'r')
            duration = sound_file.getnframes() / float(sound_file.getframerate())
            sound_file.close()
            nframes = duration * FRAMERATE
            return list(self.__random_colors(nframes))
        except IOError as e:
            rospy.logwarn("Failed to open file: " + str(e))
            return None

    def __random_colors(self, how_many):
        while how_many > 0:
            yield random.choice(COLORS)
            how_many -= 1
        return


def main():
    """ repeatedly publishes to /mouth/command, if robot is speaking """
    mouth = Mouth()
    rate = rospy.Rate(FRAMERATE)
    while not rospy.is_shutdown():
        mouth.run()
        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    main()
