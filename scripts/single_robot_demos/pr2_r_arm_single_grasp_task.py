#! /usr/bin/env python
import math
import sys
import roslib
import rospy
import sbpl_demos.pr2_helpers as pr2
import sbpl_demos.FindARTag as FindARTag

## for moveit commander
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler

# Demo code.
if __name__ == "__main__":
	rospy.init_node("simple_grasping_demo")
	
	PointHead = pr2.PointHead()
	GripperCommand = pr2.GripperCommand()
	MoveBase = pr2.MoveBase()
	# Initialize MoveIt.
	moveit_commander.roscpp_initialize(sys.argv)
	moveit_robot_commander = moveit_commander.RobotCommander()
	moveit_planning_scene = moveit_commander.PlanningSceneInterface()
	
	moveit_planning_group = moveit_commander.MoveGroupCommander("right_arm")
	moveit_planning_group.set_planner_id("RRTkConfigDefault")
	moveit_planning_group.set_planning_time(10.0)
	moveit_planning_group.allow_replanning(True)

	pose1 = geometry_msgs.msg.Pose()
	pose1.position.x = 0.09
	pose1.position.y = -0.94
	pose1.position.z = 0.89
	quat = quaternion_from_euler(0,0,0)
	pose1.orientation.x = quat[0]
	pose1.orientation.y = quat[1]
	pose1.orientation.z = quat[2]
	pose1.orientation.w = quat[3]
	
	moveit_planning_group.set_pose_target(pose1)
	plan=moveit_planning_group.plan()
	if not plan.joint_trajectory.points:
		sys.exit("No Motion Plan Found.")
	
	moveit_planning_group.go(wait=True)

	GripperCommand.Command('r', 1)

	pose2 = geometry_msgs.msg.Pose()
	pose2.position.x = 0.62
	pose2.position.y = 0
	pose2.position.z = 0.93
	quat = quaternion_from_euler(math.pi/2,0,0)
	pose2.orientation.x = quat[0]
	pose2.orientation.y = quat[1]
	pose2.orientation.z = quat[2]
	pose2.orientation.w = quat[3]

	moveit_planning_group.set_pose_target(pose2)
	plan=moveit_planning_group.plan()
	if not plan.joint_trajectory.points:
		sys.exit("No Motion Plan Found.")
	
	moveit_planning_group.go(wait=True)
	
	GripperCommand.Command('r', 0)

	# Press anykey after Roman releases the object.
	a=raw_input()

	moveit_planning_group.set_pose_target(pose1)
	plan=moveit_planning_group.plan()
	if not plan.joint_trajectory.points:
		sys.exit("No Motion Plan Found.")
	
	moveit_planning_group.go(wait=True)

	GripperCommand.Command('r', 1)

	moveit_commander.roscpp_shutdown()