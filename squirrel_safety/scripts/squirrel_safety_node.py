#! /usr/bin/env python

import rospy

from squirrel_safety import safety


if __name__ == '__main__':
    try:
        rospy.init_node('squirrel_safety_node')
        airskin_topic = rospy.get_param('~airskin_topic')
        bumper_topic = rospy.get_param('~bumper_topic')
        wrist_topic = rospy.get_param('~wrist_topic')
        safety_ = safety.SquirrelSafety(airskin_topic, bumper_topic, wrist_topic)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
