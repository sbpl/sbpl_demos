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

STATIONARY = True

if __name__ == "__main__":

	rospy.init_node('pr2_grasp_test')
	tflistener = tf.TransformListener()
	tfbroadcaster = tf.TransformBroadcaster()
	GripperCommand = pr2_helpers.GripperCommand()
	PointHead = pr2_helpers.PointHead()
	ARTagListener = perception_helpers.ARTagListener()
	PR2ARGrasping = grasping_helpers.PR2ARGrasping()
	MoveitMoveArm = pr2_helpers.MoveitMoveArm()
	TorsoCommand = pr2_helpers.TorsoCommand()
	TuckArms = pr2_helpers.TuckArms()
	MoveBase = pr2_helpers.MoveBase()
	
	rospy.sleep(1.0)
	rospy.loginfo('All Action clients connected!')

	rospy.loginfo('Commanding Untucking')
	TuckArms.UntuckArms()

	PointHead.LookAt("base_footprint", 1.25, 0, 0)

	rospy.loginfo('Commanding torso')
	TorsoCommand.MoveTorso(0.2)
	
	if(not STATIONARY):
		rospy.loginfo('Commanding Tuckarms')
		GripperCommand.Command('l', 0) #Close gripper
		TuckArms.TuckArms()

		rospy.loginfo('Commanding base to intern desk...')
		MoveBase.MoveToInternDesk()

		rospy.loginfo('Commanding Untucking')
		TuckArms.UntuckArms()

	

	while(True):
		rospy.loginfo('Commanding right gripper open')
		GripperCommand.Command('r', 1) #open grigger
		## AR TAG PICKING
		# get current EE position
		(ee_position, ee_quat) = tflistener.lookupTransform("map", "r_wrist_roll_link",rospy.Time())

		#want to latch it at this time
		(markers, n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges) = ARTagListener.getMarkersAndCounts() 
		rospy.loginfo(" Discovered %d desks, %d cylinders, %d cubes, %d rod_ends, %d cuboid_flats, %d cuboid_edges", 
			n_desks, n_cylinders, n_cubes, n_rod_ends, n_cuboid_flats, n_cuboid_edges)

		valid_poses = [] # all IK-valid poses
		best_poses = [] # the best poses, one per item
		best_poses_object = [] #TODO should be a map between pose -> object (AR) pose

		if(n_desks > 0):
			rospy.loginfo("Inserting Desk Collision objects")
			
			desk_markers = ARTagListener.getMarkersByType(markers, AR_TYPES.DESK)
			cnt = 0
			for desk in desk_markers.markers:
				
				
				desk.pose.header.frame_id = "map"
				desk.pose.header.stamp = rospy.Time.now()
				print desk
				MoveitMoveArm.AddDeskCollisionObject("desk_"+str(cnt), desk.pose)
				cnt+=1

		if(n_cylinders >0):
			rospy.loginfo("Computing poses for CYLINDERS")
			cylinder_markers = ARTagListener.getMarkersByType(markers, AR_TYPES.CYLINDER)
			cnt = 0
			for cylinder in cylinder_markers.markers:
				valid_poses_candidates = PR2ARGrasping.getValidPosesByType(cylinder.pose.pose, AR_TYPES.CYLINDER)
				if(valid_poses_candidates):
					valid_poses.append(valid_poses_candidates)
				## or ##
				best_candidate = PR2ARGrasping.getBestPoseByType(cylinder.pose.pose, AR_TYPES.CYLINDER, ee_position)
				if(best_candidate):
					best_poses.append(best_candidate)
					best_poses_object.append(cylinder.pose.pose)
				cylinder.pose.header.stamp = rospy.Time.now()
				cylinder.pose.header.frame_id = "map"

				# place a collision object at the marker
				MoveitMoveArm.AddCollisionObject("object_cylinder_" + str(cnt), cylinder.pose, (0.02, 0.02, 0.3) )
				rospy.sleep(2.0)
				cnt+=1

		if(n_cubes >0):
			rospy.loginfo("Computing poses for CUBES")
			cube_markers = ARTagListener.getMarkersByType(markers, AR_TYPES.CUBE)
			cnt = 0
			for cube in cube_markers.markers:
				valid_poses_candidates = PR2ARGrasping.getValidPosesByType(cube.pose.pose, AR_TYPES.CUBE)
				if(valid_poses_candidates):
					valid_poses.append(valid_poses_candidates)
				## or ##
				best_candidate = PR2ARGrasping.getBestPoseByType(cube.pose.pose, AR_TYPES.CUBE, ee_position)
				if(best_candidate):
					best_poses.append(best_candidate)
					best_poses_object.append(cube.pose.pose)
				cube.pose.header.stamp = rospy.Time.now()
				cube.pose.header.frame_id = "map"
				# place a collision object at the marker
				MoveitMoveArm.AddCollisionObject("object_cube_" + str(cnt), cube.pose, (0.07, 0.07, 0.07) )
				rospy.sleep(2.0)
				cnt+=1

		if(n_cuboid_edges >0):
			rospy.loginfo("Computing poses for CUBOID_EDGE")
			obj_markers = ARTagListener.getMarkersByType(markers, AR_TYPES.CUBOID_EDGE)
			cnt = 0
			for obj in obj_markers.markers:
				valid_poses_candidates = PR2ARGrasping.getValidPosesByType(obj.pose.pose, AR_TYPES.CUBOID_EDGE)
				if(valid_poses_candidates):
					valid_poses.append(valid_poses_candidates)
				## or ##
				best_candidate = PR2ARGrasping.getBestPoseByType(obj.pose.pose, AR_TYPES.CUBOID_EDGE, ee_position)
				if(best_candidate):
					best_poses.append(best_candidate)
					best_poses_object.append(obj.pose.pose)
				obj.pose.header.stamp = rospy.Time.now()
				obj.pose.header.frame_id = "map"
				# place a collision object at the marker
				MoveitMoveArm.AddCollisionObject("object_cuboid_edge_" + str(cnt), obj.pose, (0.2, 0.05, 0.02) )
				rospy.sleep(2.0)
				cnt+=1

		if(n_rod_ends >0):
			rospy.loginfo("Computing poses for ROD_END")
			obj_markers = ARTagListener.getMarkersByType(markers, AR_TYPES.ROD_END)
			cnt = 0
			for obj in obj_markers.markers:
				valid_poses_candidates = PR2ARGrasping.getValidPosesByType(obj.pose.pose, AR_TYPES.ROD_END)
				if(valid_poses_candidates):
					valid_poses.append(valid_poses_candidates)
				## or ##
				best_candidate = PR2ARGrasping.getBestPoseByType(obj.pose.pose, AR_TYPES.ROD_END, ee_position)
				if(best_candidate):
					best_poses.append(best_candidate)
					best_poses_object.append(obj.pose.pose)

				# place a collision object at the marker
				obj.pose.header.stamp = rospy.Time.now()
				obj.pose.header.frame_id = "map"
				MoveitMoveArm.AddCollisionObject("object_rod_end_" + str(cnt), obj.pose, (0.07, 0.07, 0.15) )
				rospy.sleep(2.0)
				cnt+=1

		if not len(best_poses) > 0:
			rospy.logwarn("No poses found, retrying")
			MoveitMoveArm.removeAllObjectsAndDesks()
			rospy.sleep(1)
			continue
		#somehow select desired grasp
		grasp_pose = best_poses[0] # simply pick first for now
		object_pose = best_poses_object[0]
		interp_pose = PR2ARGrasping.getInterpolatedPose(grasp_pose, object_pose)

		#just for visualization
		tfbroadcaster.sendTransform((interp_pose.position.x, interp_pose.position.y, interp_pose.position.z),
			(interp_pose.orientation.x, interp_pose.orientation.y, interp_pose.orientation.z, interp_pose.orientation.w),
			rospy.Time.now(), "desired_grasp", "map")

		# Execute grasp plan
		rospy.loginfo("Moving to interpolated pose")
		success = MoveitMoveArm.MoveToPose(interp_pose, "map")

		if not success:
			rospy.logwarn("could not move to interpolated pose, retrying")
			MoveitMoveArm.removeAllObjectsAndDesks()
			rospy.sleep(1)
			continue

		rospy.loginfo("Removing collision objects and moving to final grasp pose")
		MoveitMoveArm.removeAllObjects()
		success = MoveitMoveArm.MoveToPose(grasp_pose, "map")
		GripperCommand.Command('r', 0) #Close Gripper
		
		rospy.loginfo("Moving to wide pose")
		MoveitMoveArm.MoveRightToWide()

		if(not STATIONARY):
			#optionally move to alternate location
			rospy.loginfo('Tucking arms')
			TuckArms.TuckLeftArm()
			rospy.loginfo('Commanding base to Workstation')
			MoveBase.MoveToWorkstation()
			rospy.loginfo('Untucking arms')
			TuckArms.UntuckArms()
			#TODO clear desks, get AR poses for desk, and insert new Desk collision object

		rospy.loginfo("Moving to extend pose")
		MoveitMoveArm.MoveRightToExtend()

		rospy.loginfo("Commanding right gripper Closed")
		GripperCommand.Command('r', 1) #open gripper
		rospy.loginfo('Commanding Tuckarms')
		
		
		TuckArms.TuckLeftArm()
		GripperCommand.Command('r', 0) #Close Gripper
		MoveitMoveArm.removeAllObjectsAndDesks()

	MoveitMoveArm.Cleanup()
	print('shutting down...')	