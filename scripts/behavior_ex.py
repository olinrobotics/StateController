#!/usr/bin/env python
"""Example of a basic tractor behavior.

This module demonstrates an example behavior written in python to be
compatible with the state controller. The behavior publishes any
command received on /ex_topic directly to the corresponding topic
/state_controller/cmd_behavior.

authored by Connor Novak on 2019-10-29
"""

import rospy
from geometry_msgs.msg import Twist
from state_controller.msg import TwistLabeled

class BehaviorEx():

	def __init__(self):
		"""Initialize node, publisher and subscriber.""" 

		# In ROS, nodes are uniquely named. If two nodes with the same
		# name are launched, the previous one is kicked off. The
		# anonymous=True flag means that rospy will choose a unique
		# name for our 'listener' node so that multiple listeners can
		# run simultaneously.
		rospy.init_node('behavior_ex', anonymous=True)
		rospy.Subscriber('/ex_topic', Twist, self.callback)
		self.twist_publisher = rospy.Publisher(
			'/state_controller/cmd_behavior', TwistLabeled, queue_size=1)

		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

	def callback(self, data):
		"""Forward recieved twist message to state controller with label."""
		new_msg = TwistLabeled()
		new_msg.twist = data 
		new_msg.label.data = 'example'
		self.twist_publisher.publish(new_msg)
	
if __name__ == '__main__':
	b = BehaviorEx()
