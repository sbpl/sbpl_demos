#! /usr/bin/env python
import sys
import rospy
import actionlib
import tf
from rcta.msg import MoveArmGoal, MoveArmAction
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from roman_client_ros_utils.msg import RobotiqSimpleCmd, RobotiqActivate


class RomanMoveArm(object):
	def __init__(self):
		self.client=actionlib.SimpleActionClient('/move_arm', MoveArmAction)
		rospy.loginfo("Waiting for action /move_arm...")
		self.client.wait_for_server()
		rospy.loginfo("Connected to action /move_arm!")

	def MoveToPose(self, desired_pose):
		goal=MoveArmGoal()
		'''
		# goal 
		uint8 EndEffectorGoal = 0
		uint8 JointGoal = 1
		uint8 CartesianGoal = 2

		uint8 type
		geometry_msgs/Pose goal_pose
		sensor_msgs/JointState goal_joint_state

		moveit_msgs/RobotState start_state

		octomap_msgs/Octomap octomap

		bool execute_path

		moveit_msgs/PlanningOptions planning_options
		'''
		
		goal.type = 0 # EEgoal
		goal.goal_pose = desired_pose
		rospy.loginfo("sending robot to pose: ")
		print desired_pose

		goal.execute_path = True

		rospy.loginfo("Sending goal to action Server")
		self.client.send_goal(goal)
		result = self.client.wait_for_result()
		rospy.loginfo("Finished pose goal - result is %d", result)
		return

class RomanCommandGripper(object):
	def __init__(self):

		self.activate_pub = rospy.Publisher("roman_hand_activate", RobotiqActivate, queue_size=1)
		self.open_pub = rospy.Publisher("roman_hand_open", RobotiqSimpleCmd, queue_size=1)
		self.close_pub = rospy.Publisher("roman_hand_close", RobotiqSimpleCmd, queue_size=1)
		
		activate_msg = RobotiqActivate()
		activate_msg.activate = True
		self.activate_pub.publish(activate_msg)

	def Close(self):
		goal=RobotiqSimpleCmd()
		goal.close = True
		self.close_pub.publish(goal)
		print "Commanding Robotiq Close"
		rospy.sleep(3.0) 

	def Open(self):
		goal=RobotiqSimpleCmd()
		goal.close = False
		self.open_pub.publish(goal)
		print "Commanding Robotiq Open"
		rospy.sleep(3.0) 