#! /usr/bin/env python
import sys
import rospy
import actionlib
import tf
from sbpl_demos.roman_helpers import RomanMoveArm, RomanCommandGripper
from geometry_msgs.msg import Pose
import sbpl_demos.pr2_helpers as pr2



if __name__ == "__main__":

	rospy.init_node('roman_grasp_test')
	listener = tf.TransformListener()
	RMAC = RomanMoveArm() 
	RCG = RomanCommandGripper()
	pr2_GripperCommand = pr2.GripperCommand()
	pr2_RoconMoveArm = pr2.RoconMoveArm()

	RCG.Activate()
	RCG.Open()
	pr2_GripperCommand.GripperCommand.Command('r', 1)

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

	pr2_RoconMoveArm.MoveToHandoff()
	pr2_GripperCommand.Command('r', 0)

	RCG.Open()

	RMAC.MoveToHome()

	pr2_RoconMoveArm.MoveToHome()

	pr2_GripperCommand.Command('r', 1)

	print('shutting down...')	