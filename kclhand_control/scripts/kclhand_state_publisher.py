#!/usr/bin/env python
# Rewritten by Michael Zillich from get_fingertip_state.py
# Takes states of active joints anc calculates of passive joints, then publishes all joints.

import rospy
import math as m
import numpy as np
from std_msgs.msg import String, Float64MultiArray, MultiArrayDimension
from sensor_msgs.msg import JointState
from tf import TransformBroadcaster
import kclhand_forward_kinematics as kclhand

class kclhand_state_publisher(object):

    joint_state_pub = None
    joint_names = ["hand_left_crank_base_joint", "hand_right_crank_base_joint",
                   "hand_left_coupler_crank_joint", "hand_right_coupler_crank_joint",
                   "hand_left_finger_lower_joint", "hand_left_finger_upper_joint" ,
                   "hand_middle_finger_lower_joint", "hand_middle_finger_upper_joint",
                   "hand_right_finger_lower_joint", "hand_right_finger_upper_joint"]

    def __init__(self):
        rospy.init_node("kclhand_state_publisher", anonymous=True)
        rospy.Subscriber("active_joint_states", JointState, self.active_joint_callback)
        self.joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=10)
        rospy.spin()

    def active_joint_callback(self, msg):
        # NOTE: kclhand_fingertip_state needs angles in degrees (though internally uses radians anyway ..)
        #joint_values_deg = range(0, len(msg.position))
        #for i in range(0, len(joint_values_deg)):
        #    joint_values_deg[i] = m.degrees(msg.position[i])
        # NOTE: active joint state is given in degrees, not radians
        joint_values_deg = msg.position
        # NOTE: The minuses are right just so. It's an odd artefact of how handFK defines directions.
        fingertip_state = kclhand.HandFK(palmJointA = joint_values_deg[0],
                                         palmJointE = joint_values_deg[1],
                                         leftFingerLower = -joint_values_deg[2],
                                         middleFingerLower = -joint_values_deg[3],
                                         rightFingerLower = joint_values_deg[4])

        # NOTE: I have to call these, otherwise some variable inside HandFK is not created
        fingertip_state.rightFinger()
        fingertip_state.middleFinger()
        fingertip_state.leftFinger()

        joint_state = JointState()
        joint_state.header = msg.header
        joint_state.name = self.joint_names
        joint_state.position = fingertip_state.get_joint_displacement()
        # TODO: effort and possibly velocity of the active states
        self.joint_state_pub.publish(joint_state)


if __name__ == '__main__':
    try:
        kclhand_state_publisher()
    except rospy.ROSInterruptException:
        pass
