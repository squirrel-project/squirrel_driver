#!/usr/bin/env python

import rospy
import readchar
import sys
from std_msgs.msg import Float64MultiArray, Int32
from sensor_msgs.msg import JointState

step = 0.02 # roughly 1 degree in radian
joint_values = []

def publish(pub, joint_values):
    msg = Float64MultiArray()
    msg.layout.data_offset = 0
    msg.data = joint_values
    pub.publish(msg)

def callback(data):
    global joint_values
    joint_values = list(data.position)

def show_welcome():
    print("Welcome to the simple node to incrementally move SQUIRREL's arm and base")
    show_help()

def show_help():
    print("*"*60)
    print("First select the axis to rotate \n(0: base platform, 1-5: axis counted from bottom to top)")
    print("Second, move the axis with '+' or '-' keys. q,Q to quit.")
    print("*"*60)

def main():
    global joint_values, step
    rospy.init_node('incremental_arm_movement', anonymous=True)
    mode_pub = rospy.Publisher('/real/robotino_arm/settings/switch_mode', Int32, queue_size=10, latch=True)
    pub = rospy.Publisher('/real/robotino_arm/joint_control/move', Float64MultiArray, queue_size=10)
    rospy.Subscriber('/real/robotino_arm/joint_control/get_state', JointState, callback)
    joint = None

    rospy.loginfo('incremental_arm_movement started')
    show_welcome() 
    mode_pub.publish(data=10)
  
    while True:
        try:
            c = readchar.readchar()
            if c.lower() == 'q':
                rospy.loginfo("Quit node")
                sys.exit()
            elif c.lower() == 'h':
                show_help()
            elif c == '+' and joint is not None:
                rospy.loginfo("joint #{}: plus".format(joint))
                joint_values[joint] += step
                publish(pub, joint_values)
            elif c == '-' and joint is not None:
                rospy.loginfo("joint #{}: minus".format(joint))
                joint_values[joint] -= step
                publish(pub, joint_values)
            elif abs(int(c)) <= 5:
                rospy.loginfo(c)
                joint = int(c)
            else:
                rospy.loginfo("unknown")
                pass
        except ValueError as e:
            #print(e)
            print("Command not recognized. Press 'h' for instructions")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
