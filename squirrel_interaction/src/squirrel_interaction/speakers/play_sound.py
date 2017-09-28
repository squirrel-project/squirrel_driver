#!/usr/bin/env python

"""

This ROS Node provides an interface for the Speakers connected to the Raspberry Pi.

It makes use of the aplay application to do so.

The service /sound/play is called with the name of the soundfile to be played.

All .wav files under $(find squirrel_interaction)/resources/sounds are available.

"""

import os
import rospy
import rospkg
from squirrel_interaction.srv import *

ROSPACK = rospkg.RosPack()
PATH = ROSPACK.get_path('squirrel_interaction')

def play(data):
    """ Plays the sound file with the name passed as parameter """
    try:
        comand = "aplay -q " + PATH + "/resources/sounds/" + data.message + ".wav"
        os.system(comand)
        return True
    except OSError as exc:
        rospy.logerr("Failed to play sound file. " + exc.message)

def get_sounds(data):
    """ lists all robot sounds
    """
    files_in_dir = os.listdir(PATH + "/resources/sounds")
    expressions = __filter_wav_files(files_in_dir)
    names = __remove_extensions(expressions)
    return GetListResponse(names)

def __filter_wav_files(files):
    for curr_file in files:
        if curr_file.endswith(".wav"):
            yield curr_file

def __remove_extensions(files):
    return [filename[:-4] for filename in files]


def _listener():
    rospy.init_node('playsound')
    rospy.Service('sound/play', PlaySound, play)
    rospy.Service("sound/sounds", GetList, get_sounds)
    rospy.loginfo("Ready to Play Sounds")
    rospy.spin()

if __name__ == '__main__':
    _listener()
