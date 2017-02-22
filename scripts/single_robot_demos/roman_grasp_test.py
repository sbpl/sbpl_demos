#! /usr/bin/env python
import sys
import rospy
import actionlib
import tf
from sbpl_demos.roman_helpers import RomanMoveArm, RomanCommandGripper
from geometry_msgs.msg import Pose


if __name__ == "__main__":

	rospy.init_node('roman_grasp_test')
	RMAC = RomanMoveArm() 
	RCG = RomanCommandGripper()

	RCG.Open()

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

	RCG.Close()

	print('shutting down...')	