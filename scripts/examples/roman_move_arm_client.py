#! /usr/bin/env python
import sys
import rospy
import actionlib
import tf
from rcta.msg import MoveArmGoal, MoveArmAction
from geometry_msgs.msg import Pose
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState


class RomanMoveArmClient(object):
	def __init__(self):
		self.client=actionlib.SimpleActionClient('/roman/move_arm', MoveArmAction)
		rospy.loginfo("Waiting for action /roman/move_arm...")
		self.client.wait_for_server()
		rospy.loginfo("Connected to action /roman/move_arm!")
		self.joint_state = JointState()

		self.joint_state_sub = rospy.Subscriber("roman_joint_states", JointState, self.jointStateCB)


	def jointStateCB(self, data):
		self.joint_state = data

	def MoveToPose(self, desired_pose):
		#rospy.spinOnce() ## Check for latest joint state callback
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

		start_state = RobotState()
		
		start_state.joint_state = self.joint_state
		start_state.joint_state.header.frame_id = "map"
		start_state.is_diff = True
		goal.start_state = start_state
		print goal.start_state

		goal.execute_path = True

		rospy.loginfo("Sending goal to action Server")
		self.client.send_goal(goal)
		result = self.client.wait_for_result()
		rospy.loginfo("Finished pose goal - result is %d", result)
		return


	def run(self):
		rospy.spin()


if __name__ == "__main__":

	rospy.init_node('roman_move_arm_client_test')
	RMAC = RomanMoveArmClient() 

	listener = tf.TransformListener()
	rospy.sleep(3.0)
	if listener.frameExists("roman_ar_grasp"):
		grasp_tf = listener.lookupTransform("base_footprint","roman_ar_grasp",  rospy.Time())
		print grasp_tf
	else:
		print "could not find grasp frame"
		sys.exit
	grasp_pose = Pose()
	grasp_pose.position.x = grasp_tf[0][0]
	grasp_pose.position.y = grasp_tf[0][1]
	grasp_pose.position.z = grasp_tf[0][2]

	grasp_pose.orientation.x = grasp_tf[1][0]
	grasp_pose.orientation.y = grasp_tf[1][1]
	grasp_pose.orientation.z = grasp_tf[1][2]
	grasp_pose.orientation.w = grasp_tf[1][3]


	

	RMAC.MoveToPose(grasp_pose)
	print('shutting down...')	