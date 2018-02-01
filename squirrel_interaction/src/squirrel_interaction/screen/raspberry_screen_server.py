#!/usr/bin/env python

"""

This ROS Node provides an interface for the Monitor attached to the Raspberry Pi.

The ros param face_color defines subfolder to look into.

All .png files under $(find squirrel_interaction)/resources/media/images/$(face_color) are available.

"""

import os
import cv2
import rospy
import rospkg
from squirrel_interaction.srv import *
from std_msgs.msg import String

ROSPACK = rospkg.RosPack()
PATH = ROSPACK.get_path('squirrel_interaction')
BASE_COLOR = rospy.get_param("face_color", default="navy")

def emotion_cb(msg):
    global img_file
    base_color = rospy.get_param("face_color", default="gold")
    img_file= "%s/resources/media/images/%s/%s.png" % (PATH, base_color, msg.data)

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
    global img_file
    rospy.init_node('raspberry_screen_server')
    rospy.Subscriber("/face/emotion", String, emotion_cb)
    rospy.Service('face/emotions', GetList, get_available_expressions)
    cv2.namedWindow("image", cv2.WND_PROP_FULLSCREEN)          
    cv2.setWindowProperty("image", cv2.WND_PROP_FULLSCREEN, cv2.cv.CV_WINDOW_FULLSCREEN)

    img_file= "{}/resources/media/images/{}/{}.png".format(PATH, "gold", "look_down")
    rospy.loginfo("Ready to display on screen")

    r = rospy.Rate(1)
    while not rospy.is_shutdown():
        image = cv2.imread(img_file)
        image = cv2.resize(image, (1280,720))
        cv2.imshow('image', image)
        cv2.waitKey(20)
        r.sleep()
    rospy.spin()


if __name__ == "__main__":
    main()
