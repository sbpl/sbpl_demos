#! /usr/bin/env python
import sys
import rospy
import actionlib
import tf
from sbpl_demos.roman_helpers import RomanMoveitCommander, RomanCommandGripper
from geometry_msgs.msg import Pose


if __name__ == "__main__":

	rospy.init_node('roman_grasp_test')
	listener = tf.TransformListener()
	RMAC = RomanMoveitCommander() 
	RCG = RomanCommandGripper()

	RCG.Activate()
	RCG.Open()

	RMAC.MoveToHome()

	if listener.frameExists("roman_ar_pregrasp"):
		grasp_tf = listener.lookupTransform("map","roman_ar_pregrasp",  rospy.Time())
		print grasp_tf
	else:
		print "could not find grasp frame"
		sys.exit
	pregrasp_pose = Pose()
	pregrasp_pose.position.x = grasp_tf[0][0]
	pregrasp_pose.position.y = grasp_tf[0][1]
	pregrasp_pose.position.z = grasp_tf[0][2]

	pregrasp_pose.orientation.x = grasp_tf[1][0]
	pregrasp_pose.orientation.y = grasp_tf[1][1]
	pregrasp_pose.orientation.z = grasp_tf[1][2]
	pregrasp_pose.orientation.w = grasp_tf[1][3]

	
	if listener.frameExists("roman_ar_grasp"):
		grasp_tf = listener.lookupTransform("map","roman_ar_grasp",  rospy.Time())
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

	RMAC.MoveToPose(pregrasp_pose)

	RMAC.MoveToPose(grasp_pose)

	RCG.Pinch()

	RMAC.MoveToHandoff()

	key_input = raw_input("Proceed? [y/n]")

	RCG.Open()

	RMAC.MoveToHome()

	RMAC.Cleanup()
	print('shutting down...')	