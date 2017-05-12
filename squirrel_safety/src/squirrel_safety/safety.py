import rospy

from squirrel_safety_msgs.msg import Safety
from std_msgs.msg import Bool

class SquirrelSafety(object):
    def __init__(self, airskin_topic, bumper_topic, wrist_topic):
        while rospy.get_time() == 0.0:
            pass

        rospy.loginfo(rospy.get_caller_id() + ': starting up')

        self.wrist_sub = rospy.Subscriber(wrist_topic, Bool, self.wrist_callback)
        self.bumper_sub = rospy.Subscriber(bumper_topic, Bool, self.bumper_callback)
        self.airskin_sub = rospy.Subscriber(airskin_topic, Bool, self.airskin_callback)
        self.safety_pub = rospy.Publisher('/squirrel_safety', Safety, queue_size=10)

        rate = rospy.Rate(10)
        self.safety_msg = Safety()
        self.safety_msg.emergency_state = False
        self.safety_msg.airskin_stop = False
        self.safety_msg.wrist_stop = False
        self.safety_msg.bumper_stop = False

        while not rospy.is_shutdown():
            if self.safety_msg.wrist_stop or self.safety_msg.bumper_stop or self.safety_msg.airskin_stop:
                self.safety_msg.emergency_state = True
            self.safety_pub.publish(self.safety_msg)
            self.safety_msg.emergency_state = False
            rate.sleep()


    def airskin_callback(self, msg):
        if msg:
            self.safety_msg.airskin_stop = True
        else:
            self.safety_msg.airskin_stop = False


    def wrist_callback(self, msg):
        if msg:
            self.safety_msg.wrist_stop = True
        else:
            self.safety_msg.wrist_stop = False


    def bumper_callback(self, msg):
        if msg:
            self.safety_msg.bumper_stop = True
        else:
            self.safety_msg.bumper_stop = False
