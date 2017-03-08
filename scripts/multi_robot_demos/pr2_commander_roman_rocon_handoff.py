#! /usr/bin/env python
import sys
import math
import copy
import rospy
import actionlib
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, PoseStamped
from sbpl_demos import perception_helpers
from sbpl_demos import grasping_helpers
from sbpl_demos import pr2_helpers
from sbpl_demos.perception_helpers import AR_TYPES
from sbpl_demos.msg import RoconMoveArmAction, RoconMoveArmGoal, RoconMoveArmResult
from sbpl_demos.msg import RoconPickAction, RoconPickGoal, RoconPickResult
from sbpl_demos.msg import RoconRobotiqAction, RoconRobotiqGoal, RoconRobotiqResult


class Demo:
	def __init__(self):
		self.STATIONARY = False
		self.tflistener = tf.TransformListener()
		self.tfbroadcaster = tf.TransformBroadcaster()
		self.GripperCommand = pr2_helpers.GripperCommand()
		self.PointHead = pr2_helpers.PointHead()
		self.ARTagListener = perception_helpers.ARTagListener()
		self.PR2ARGrasping = grasping_helpers.PR2ARGrasping()
		self.MoveitMoveArm = pr2_helpers.MoveitMoveArm()
		self.TorsoCommand = pr2_helpers.TorsoCommand()
		self.TuckArms = pr2_helpers.TuckArms()
		self.MoveBase = pr2_helpers.MoveBase()
		self.PickClient=actionlib.SimpleActionClient('/roman_pick_action', RoconPickAction)
		self.ArmClient=actionlib.SimpleActionClient('/roman_move_arm', RoconMoveArmAction)
		self.GraspClient=actionlib.SimpleActionClient('/roman_grasp_action', RoconRobotiqAction)
		rospy.loginfo("Waiting for servers...")
		self.PickClient.wait_for_server()
		self.ArmClient.wait_for_server()
		self.GraspClient.wait_for_server()
		rospy.loginfo("Connected.")
	
		rospy.sleep(1.0)
		rospy.loginfo('All Action clients connected!')
		#rospy.loginfo('Commanding Untucking')
		#self.TuckArms.UntuckArms()
		self.PointHead.LookAt("base_footprint", 1.25, 0, 0)
		rospy.loginfo('Commanding torso')
		self.TorsoCommand.MoveTorso(0.2)

	def RomanMoveToHome(self):
		Home = RoconMoveArmGoal()
		Home.goal_type = RoconMoveArmGoal.HOME
		print "go home"
		self.ArmClient.send_goal(Home)
		self.ArmClient.wait_for_result(rospy.Duration(25.0))

	def RomanMoveToHandoff(self):
		Handoff = RoconMoveArmGoal()
		Handoff.goal_type = RoconMoveArmGoal.HANDOFF
		print "go to hanooff"
		self.ArmClient.send_goal(Handoff)
		self.ArmClient.wait_for_result(rospy.Duration(25.0))

	def RomanMoveToPostHandoff(self):
		pose = Pose()
		pose.position.x = 0.711
		pose.position.y = -0.608
		pose.position.z = 1.167
		quat = quaternion_from_euler(3.14, 0.741, -2.708)
		pose.orientation.x = quat[0]
		pose.orientation.y = quat[1]
		pose.orientation.z = quat[2]
		pose.orientation.w = quat[3]
		goal = RoconMoveArmGoal()
		goal.goal_type = RoconMoveArmGoal.POSE 
		goal.target_pose = pose
		self.ArmClient.send_goal(goal)
		self.ArmClient.wait_for_result(rospy.Duration(25.0))

	def RomanGripper(self, state):
		if state == 1:
			Open = RoconRobotiqGoal()
			Open.grasp_type = RoconRobotiqGoal.OPEN
			self.GraspClient.send_goal(Open)
		else:
			Close = RoconRobotiqGoal()
			Close.grasp_type = RoconRobotiqGoal.CLOSE
			self.GraspClient.send_goal(Close)

	def RomanPick(self):
		print "pick"
		self.PickClient.send_goal(RoconPickGoal())
		result = self.PickClient.wait_for_result()

	def computeObjectPoseRoutine(self, name, artype, ee_position, markers):
		obj_markers = self.ARTagListener.getMarkersByType(markers, artype)
		cnt = 0
		for obj in obj_markers.markers:
			valid_poses_candidates = self.PR2ARGrasping.getValidPosesByType(obj.pose.pose, artype)
			if(valid_poses_candidates):
				self.valid_poses.append(valid_poses_candidates)
			## or ##
			best_candidate = self.PR2ARGrasping.getBestPoseByType(obj.pose.pose, artype, ee_position)
			if(best_candidate):
				self.best_poses.append(best_candidate)
				self.best_poses_object.append(obj.pose.pose)
			
			# place a collision object at the marker
			obj.pose.header.stamp = rospy.Time.now()
			obj.pose.header.frame_id = "map"
			size = (0,0,0)
			if(artype == AR_TYPES.CUBOID_EDGE):
				size = (0.2, 0.05, 0.05)
			elif(artype == AR_TYPES.CYLINDER):
				size = (0.02, 0.02, 0.3)
			elif(artype == AR_TYPES.ROD_END):
				size = (0.07, 0.07, 0.15)
			elif(artype == AR_TYPES.CUBE):
				size = (0.07, 0.07, 0.07)
			self.MoveitMoveArm.AddCollisionObject("object_"+name+"_"+ str(cnt), obj.pose, size )
			rospy.sleep(2.0)
			cnt+=1

	def moveToWorkstationRoutine(self):
		if(not self.STATIONARY):
				#optionally move to alternate location
				rospy.loginfo('Tucking arms')
				self.TuckArms.TuckLeftArm()
				rospy.loginfo("Moving to wide pose")
				self.MoveitMoveArm.MoveRightToCarry()
				rospy.loginfo('Commanding base to Workstation')
				self.MoveBase.MoveToWorkstation()
				rospy.loginfo('Untucking arms')
				self.TuckArms.UntuckArms()

	def moveToInternDeskRoutine(self):
		if(not self.STATIONARY):
			rospy.loginfo('Commanding Tuckarms')
			self.GripperCommand.Command('l', 0) #Close gripper
			self.TuckArms.TuckArms()

			rospy.loginfo('Commanding base to intern desk...')
			self.MoveBase.MoveToInternDesk()

			rospy.loginfo('Commanding Untucking')
			self.TuckArms.UntuckArms()

	def pickingRoutine(self):
		rospy.loginfo('Commanding right gripper open')
		self.GripperCommand.Command('r', 1) #open grigger
		## AR TAG PICKING
		# get current EE position
		(ee_position, ee_quat) = self.tflistener.lookupTransform("map", "r_wrist_roll_link",rospy.Time())

		#want to latch it at this time
		(markers, n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges) = self.ARTagListener.getMarkersAndCounts() 
		rospy.loginfo(" Discovered %d desks, %d cylinders, %d cubes, %d rod_ends, %d cuboid_flats, %d cuboid_edges", 
			n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges)

		self.valid_poses = [] # all IK-valid poses
		self.best_poses = [] # the best poses, one per item
		self.best_poses_object = [] #TODO should be a map between pose -> object (AR) pose

		if(n_desks > 0):
			rospy.loginfo("Inserting Desk Collision objects")
			desk_markers = self.ARTagListener.getMarkersByType(markers, AR_TYPES.DESK)
			cnt = 0
			for desk in desk_markers.markers:
				desk.pose.header.frame_id = "map"
				desk.pose.header.stamp = rospy.Time.now()
				print desk
				self.MoveitMoveArm.AddDeskCollisionObject("desk_"+str(cnt), desk.pose)
				cnt+=1

		if(n_cylinders >0):
			rospy.loginfo("Computing poses for CYLINDERS")
			self.computeObjectPoseRoutine("cylinder", AR_TYPES.CYLINDER, ee_position, markers)

		if(n_cubes >0):
			rospy.loginfo("Computing poses for CUBES")
			self.computeObjectPoseRoutine("cubes", AR_TYPES.CUBE, ee_position, markers)

		if(n_cuboid_edges >0):
			rospy.loginfo("Computing poses for CUBOID_EDGE")
			self.computeObjectPoseRoutine("cuboid_edge", AR_TYPES.CUBOID_EDGE, ee_position, markers)			

		if(n_rod_ends >0):
			rospy.loginfo("Computing poses for ROD_END")
			self.computeObjectPoseRoutine("rod_end", AR_TYPES.ROD_END, ee_position, markers)

	def dropOffObjectRoutine(self):
		rospy.loginfo("Moving to extend pose")
		self.MoveitMoveArm.MoveRightToExtend()
		rospy.loginfo("Commanding right gripper Closed")
		self.GripperCommand.Command('r', 1) #open gripper
		self.MoveitMoveArm.MoveRightToShortExtend()
		rospy.loginfo('Commanding Tuckarms')
		self.TuckArms.TuckLeftArm()

	def Pr2MoveToRomanHandoff(self):
		pose = Pose()
		pose.position.x = -0.049
		pose.position.y = -0.824
		pose.position.z = 1.163
		quat = quaternion_from_euler(1.571, 0, -2.188)
		pose.orientation.x = quat[0]
		pose.orientation.y = quat[1]
		pose.orientation.z = quat[2]
		pose.orientation.w = quat[3]
		self.MoveitMoveArm.MoveToPose(pose, "base_footprint")

	def Pr2MoveToRomanHandoff(self):
		pose = Pose()
		pose.position.x = -0.049
		pose.position.y = -0.824
		pose.position.z = 1.163
		quat = quaternion_from_euler(1.571, 0, -2.188)
		pose.orientation.x = quat[0]
		pose.orientation.y = quat[1]
		pose.orientation.z = quat[2]
		pose.orientation.w = quat[3]
		self.MoveitMoveArm.MoveToPose(pose, "base_footprint")

	def Pr2MoveToRomanPostHandoff(self):
		pose = Pose()
		pose.position.x = 0.172
		pose.position.y = -0.57
		pose.position.z = 1.14
		quat = quaternion_from_euler(1.535, -0.022, -2.458)
		pose.orientation.x = quat[0]
		pose.orientation.y = quat[1]
		pose.orientation.z = quat[2]
		pose.orientation.w = quat[3]
		self.MoveitMoveArm.MoveToPose(pose, "base_footprint")

	def runDemo(self):
		while not rospy.is_shutdown():

			self.RomanGripper(1)
			self.RomanMoveToHome()
			self.RomanPick()

			self.RomanMoveToHandoff()

			self.GripperCommand.Command('r', 1) #open
			self.Pr2MoveToRomanHandoff()
			self.GripperCommand.Command('r', 0) #Close Gripper
			self.RomanGripper(0)
			rospy.sleep(2)
			#self.RomanMoveToPostHandoff()
			self.Pr2MoveToRomanPostHandoff()
			self.RomanMoveToHome()
			self.dropOffObjectRoutine()
			
			self.GripperCommand.Command('r', 0) #Close Gripper

		self.MoveitMoveArm.Cleanup()
		print('shutting down...')	


if __name__ == "__main__":
	rospy.init_node('pr2_grasp_test')
	demo = Demo()
	demo.runDemo()

