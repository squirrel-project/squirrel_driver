#!/usr/bin/env python

"""

This ROS Node provides an interface for the Monitor attached to the Raspberry Pi.

It makes use of the Eye of Gnome (EOG)  application to do so.

The service /face/emotion is called with the name of the image to be displayed without extention.

All .png files under $(find squirrel_interaction)/resources/media/images are available.

"""

import os
import rospy
import rospkg
from squirrel_interaction.srv import *

ROSPACK = rospkg.RosPack()
PATH = ROSPACK.get_path('squirrel_interaction')
BASE_COLOR = rospy.get_param("face_color", default="navy")

def handle_display_on_screen(data):
    """ If message passed to service is "stop", kill node and EOG.

        If file exists, displays it.
    """
    try:
        base_color = rospy.get_param("face_color", default="navy")
        if data.message == "stop":
            os.system("kill $(pidof eog) &")
            rospy.signal_shutdown('EOG application node closed. Reason: User Command')
        else:
            to_open = "%s/resources/media/images/%s/%s.png" % (PATH, base_color, data.message)
            os.system('eog --fullscreen -w "' + to_open + '" &')
        return True

    except rospy.ServiceException as exc:
        print "Service did not process request: " + str(exc)
        return False

def get_available_expressions(data):
    """ lists all face names
    """
    base_color = rospy.get_param("face_color", default="navy")
    files_in_dir = os.listdir(PATH + "/resources/media/images/" + base_color)

    expressions = __filter_png_files(files_in_dir)
    names = __remove_extensions(expressions)
    return GetListResponse(names)

def __filter_png_files(files):
    for curr_file in files:
        if curr_file.endswith(".png"):
            yield curr_file

def __remove_extensions(files):
    return [filename[:-4] for filename in files]


def main():
    rospy.init_node('raspberry_screen_server')
    rospy.Service('face/emotion', DisplayScreen, handle_display_on_screen)
    rospy.Service('face/emotions', GetList, get_available_expressions)
    rospy.loginfo("Ready to display on screen")
    rospy.spin()


if __name__ == "__main__":
    main()
