#! /usr/bin/env python
import sys
import rospy
import actionlib
import tf
from sbpl_demos.roman_helpers import RomanMoveArm, RomanCommandGripper
import sbpl_demos.grasping_helpers as grasping_helpers
import sbpl_demos.perception_helpers as perception_helpers
from sbpl_demos.perception_helpers import AR_TYPES
from geometry_msgs.msg import Pose


class RomanDemo:
	def __init__(self):

		self.tflistener = tf.TransformListener()
		self.tfbroadcaster = tf.TransformBroadcaster()
		self.RMAC = RomanMoveArm() 
		self.RCG = RomanCommandGripper()
		self.RomanARGrasping = grasping_helpers.RomanARGrasping()
		self.ARTagListener = perception_helpers.ARTagListener()

	def pickingRoutine(self):
		## AR TAG PICKING
		# get current EE position
		(ee_position, ee_quat) = self.tflistener.lookupTransform("map", "limb_right_link7",rospy.Time())

		#want to latch it at this time
		(markers, n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges) = self.ARTagListener.getMarkersAndCounts() 
		rospy.loginfo(" Discovered %d desks, %d cylinders, %d cubes, %d rod_ends, %d cuboid_flats, %d cuboid_edges", 
			n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges)

		self.valid_poses = [] # all IK-valid poses
		self.best_poses = [] # the best poses, one per item
		self.best_poses_object = [] #TODO should be a map between pose -> object (AR) pose

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

	def computeObjectPoseRoutine(self, name, artype, ee_position, markers):
		obj_markers = self.ARTagListener.getMarkersByType(markers, artype)
		cnt = 0
		for obj in obj_markers.markers:
			valid_poses_candidates = self.RomanARGrasping.getValidPosesByType(obj.pose.pose, artype)
			if(valid_poses_candidates):
				self.valid_poses.append(valid_poses_candidates)
			## or ##
			best_candidate = self.RomanARGrasping.getBestPoseByType(obj.pose.pose, artype, ee_position)
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
			#self.RMAC.AddCollisionObject("object_"+name+"_"+ str(cnt), obj.pose, size )
			rospy.sleep(2.0)
			cnt+=1

	def runDemo(self):

		self.RCG.Activate()
		self.RCG.Open()
		self.RMAC.MoveToHome()
		while not rospy.is_shutdown():
			self.RMAC.MoveToHome()
			self.pickingRoutine()

			if not len(self.best_poses) > 0:
				rospy.logwarn("No poses found, retrying")
				rospy.sleep(1)
				continue
			#somehow select desired grasp
			grasp_pose = self.best_poses[0] # simply pick first for now
			object_pose = self.best_poses_object[0]
			interp_pose = self.RomanARGrasping.getInterpolatedPose(grasp_pose, object_pose)

			#just for visualization
			self.tfbroadcaster.sendTransform((interp_pose.position.x, interp_pose.position.y, interp_pose.position.z),
				(interp_pose.orientation.x, interp_pose.orientation.y, interp_pose.orientation.z, interp_pose.orientation.w),
				rospy.Time.now(), "desired_grasp", "map")

			# Execute grasp plan
			rospy.loginfo("Moving to interpolated pose")
			success = self.RMAC.MoveToPose(interp_pose, "map")

			if not success:
				rospy.logwarn("could not move to interpolated pose, retrying")
				rospy.sleep(1)
				continue

			rospy.loginfo("moving to final grasp pose")
			success = self.RMAC.MoveToPose(grasp_pose, "map")
			self.RCG.Close()

			self.RMAC.MoveToHandoff()

			key_input = raw_input("Proceed? [y/n]")

			self.RCG.Open()

			self.RMAC.MoveToHome()



if __name__ == "__main__":
	rospy.init_node('roman_grasp_test_v2')
	demo = RomanDemo()
	demo.runDemo()
