#! /usr/bin/env python
import sys
import rospy
import actionlib
import tf
from geometry_msgs.msg import Pose, PoseStamped
import sbpl_demos.pr2_helpers as pr2

def lookupPoseFromTransform(source_frame, target_frame):
	if listener.frameExists("target_frame"):
		grasp_tf = listener.lookupTransform(source_frame, target_frame,  rospy.Time())
		print grasp_tf
	else:
		print "could not find grasp frame"
		return False
	pose = Pose()
	pose.position.x = grasp_tf[0][0]
	pose.position.y = grasp_tf[0][1]
	pose.position.z = grasp_tf[0][2]

	pose.orientation.x = grasp_tf[1][0]
	pose.orientation.y = grasp_tf[1][1]
	pose.orientation.z = grasp_tf[1][2]
	pose.orientation.w = grasp_tf[1][3]

	return pose

if __name__ == "__main__":

	rospy.init_node('pr2_grasp_test')
	listener = tf.TransformListener()
	pr2_GripperCommand = pr2.GripperCommand()
	pr2_MoveitMoveArm = pr2.MoveitMoveArm()
	pr2_TorsoCommand = pr2.TorsoCommand()
	pr2_TuckArms = pr2.TuckArms()
	pr2_MoveBase = pr2.MoveBase()
	rospy.loginfo('All Action clients connected!')

	pr2_TorsoCommand.MoveTorso(0.15)
	pr2_GripperCommand.Command('r', 1)	

	print "moving pr2 to handoff"
	pr2_MoveitMoveArm.MoveToHandoff()
	pr2_GripperCommand.Command('r', 0)

	print "moving pr2 to home"
	pr2_MoveitMoveArm.MoveToHome()

	pr2_GripperCommand.Command('r', 1)

	pr2_MoveitMoveArm.Cleanup()
	print('shutting down...')	