import rospy

from squirrel_safety_msgs.msg import Safety
from std_msgs.msg import Bool

from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest

class SquirrelSafety(object):
    def __init__(self, airskin_topic, bumper_topic, wrist_topic):
        while rospy.get_time() == 0.0:
            pass

        rospy.loginfo(rospy.get_caller_id() + ': starting up')

        self.wrist_sub = rospy.Subscriber(wrist_topic, Bool, self.wrist_callback)
        self.bumper_sub = rospy.Subscriber(bumper_topic, Bool, self.bumper_callback)
        self.airskin_sub = rospy.Subscriber(airskin_topic, Bool, self.airskin_callback)
        self.safety_pub = rospy.Publisher('/squirrel_safety', Safety, queue_size=10)
	self.safety_reset_sub = rospy.Subscriber('/squirrel_safety/reset', Bool, self.safety_reset_callback)
        self.controller_reset_ = rospy.ServiceProxy('/arm_controller/controller_manager/switch_controller', SwitchController)

        rate = rospy.Rate(10)
        self.safety_msg = Safety()
        self.safety_msg.emergency_state = False
        self.safety_msg.airskin_stop = False
        self.safety_msg.wrist_stop = False
        self.safety_msg.bumper_stop = False

        while not rospy.is_shutdown():
            if self.safety_msg.wrist_stop or self.safety_msg.bumper_stop or self.safety_msg.airskin_stop:
                self.safety_msg.emergency_state = True
	        req = SwitchControllerRequest()
	        req.start_controllers = []
	        req.stop_controllers = ['joint_trajectory_controller']
	        req.strictness = 2
	        if self.controller_reset_(req):
	    	    rospy.loginfo('Safety set.')
	        else:
		    rospy.logerror('Failed to set safety. Please kick it again or manually shut down and panic!')
            
	    self.safety_pub.publish(self.safety_msg)
            self.safety_msg.emergency_state = False
            rate.sleep()


    def safety_reset_callback(self, msg):
	if msg.data == True:
	    req = SwitchControllerRequest()
	    req.stop_controllers = []
	    req.start_controllers = ['joint_trajectory_controller']
	    req.strictness = 2
	    if self.controller_reset_(req):
		rospy.loginfo('Safety released.')
	    else:
		rospy.logerror('Failed to release safety. Please try again or manually restart both the controller and the safety node!')
	    


    def airskin_callback(self, msg):
        self.safety_msg.airskin_stop = msg.data


    def wrist_callback(self, msg):
        self.safety_msg.wrist_stop = msg.data


    def bumper_callback(self, msg):
        self.safety_msg.bumper_stop = msg.data
