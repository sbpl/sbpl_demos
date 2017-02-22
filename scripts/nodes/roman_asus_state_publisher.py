#! /usr/bin/env python
import sys
import rospy
import tf
import dynamixel_msgs.msg as dyn_msg 
import sensor_msgs.msg as sm_msg

class DynamixelStatePublisher(object):
	def __init__(self):

		self.dyn_state = dyn_msg.JointState()
		self.sm_state = sm_msg.JointState()
		self.joint_state_sub = rospy.Subscriber("pan_tilt_0/state", dyn_msg.JointState, self.jointStateCB)
		self.pub = rospy.Publisher("dynamixel_joint_state", sm_msg.JointState, queue_size=1)

	def jointStateCB(self, data):
		self.dyn_state = data
		self.sm_state.header = self.dyn_state.header
		self.sm_state.header.frame_id = "chest"
		self.sm_state.name = ["dynamixel_joint"]
		self.sm_state.position = [self.dyn_state.current_pos]
		self.sm_state.velocity = [0]
		self.sm_state.effort = [0]

		print self.sm_state


	def run(self):
		while not rospy.is_shutdown():
			self.pub.publish(self.sm_state)
			rospy.sleep(.01)



if __name__ == "__main__":

	rospy.init_node('roman_asus_state_publisher')
	DSP = DynamixelStatePublisher()
	DSP.run()

