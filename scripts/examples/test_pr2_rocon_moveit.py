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
	pr2_GripperCommand = pr2.GripperCommand()
	pr2_RoconMoveArm = pr2.RoconMoveArm()

	pr2_GripperCommand.GripperCommand.Command('r', 1)

	pr2_RoconMoveArm.MoveToHandoff()
	pr2_GripperCommand.Command('r', 0)

	pr2_RoconMoveArm.MoveToHome()

	pr2_GripperCommand.Command('r', 1)

	print('shutting down...')	