#! /usr/bin/env python
import sys
import math
import copy
import rospy
import actionlib
import tf
from geometry_msgs.msg import Pose, PoseStamped
from sbpl_demos import perception_helpers
from sbpl_demos import grasping_helpers
from sbpl_demos import pr2_helpers
from sbpl_demos.perception_helpers import AR_TYPES


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
	
		rospy.sleep(1.0)
		rospy.loginfo('All Action clients connected!')
		#rospy.loginfo('Commanding Untucking')
		#self.TuckArms.UntuckArms()
		self.PointHead.LookAt("base_footprint", 1.25, 0, 0)
		rospy.loginfo('Commanding torso')
		self.TorsoCommand.MoveTorso(0.2)

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

	def runDemo(self):
		while not rospy.is_shutdown():
			
			self.moveToInternDeskRoutine()

			self.pickingRoutine()

			if not len(self.best_poses) > 0:
				rospy.logwarn("No poses found, retrying")
				self.MoveitMoveArm.removeAllObjectsAndDesks()
				rospy.sleep(1)
				continue
			#somehow select desired grasp
			grasp_pose = self.best_poses[0] # simply pick first for now
			object_pose = self.best_poses_object[0]
			interp_pose = self.PR2ARGrasping.getInterpolatedPose(grasp_pose, object_pose)

			#just for visualization
			self.tfbroadcaster.sendTransform((interp_pose.position.x, interp_pose.position.y, interp_pose.position.z),
				(interp_pose.orientation.x, interp_pose.orientation.y, interp_pose.orientation.z, interp_pose.orientation.w),
				rospy.Time.now(), "desired_grasp", "map")

			# Execute grasp plan
			rospy.loginfo("Moving to interpolated pose")
			success = self.MoveitMoveArm.MoveToPose(interp_pose, "map")

			if not success:
				rospy.logwarn("could not move to interpolated pose, retrying")
				self.MoveitMoveArm.removeAllObjectsAndDesks()
				rospy.sleep(1)
				continue

			rospy.loginfo("Removing collision objects and moving to final grasp pose")
			self.MoveitMoveArm.removeAllObjects()
			success = self.MoveitMoveArm.MoveToPose(grasp_pose, "map")
			self.GripperCommand.Command('r', 0) #Close Gripper
			
			rospy.loginfo("Moving to carry pose")
			self.MoveitMoveArm.MoveRightToWide()

			self.moveToWorkstationRoutine()
			#TODO clear desks, get AR poses for desk, and insert new Desk collision object

			self.dropOffObjectRoutine()
			
			self.GripperCommand.Command('r', 0) #Close Gripper
			self.MoveitMoveArm.removeAllObjectsAndDesks()

		self.MoveitMoveArm.Cleanup()
		print('shutting down...')	


if __name__ == "__main__":
	rospy.init_node('pr2_grasp_test')
	demo = Demo()
	demo.runDemo()

