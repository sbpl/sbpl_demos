#! /usr/bin/env python
import sys
import roslib
import rospy

## for moveit commander
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import tf
from geometry_msgs.msg import *

# designated orientation: x=0 y=0 z=0 w=1 
if __name__ == "__main__":
	if len(sys.argv) < 4:
		print "Usage: ./pr2_move_r_arm_to_target.py [x coordinate] [y cooridnate] [z coordinate]"
		sys.exit()

	x=float(sys.argv[1])
	y=float(sys.argv[2])
	z=float(sys.argv[3])

	rospy.init_node("pr2_move_r_arm_to_target")
	
	# Initialize MoveIt.
	moveit_commander.roscpp_initialize(sys.argv)
	moveit_robot_commander = moveit_commander.RobotCommander()
	moveit_planning_scene = moveit_commander.PlanningSceneInterface()
	moveit_planning_group = moveit_commander.MoveGroupCommander("right_arm")
	moveit_planning_group.set_planner_id("RRTkConfigDefault")
	moveit_planning_group.set_planning_time(10.0)
	moveit_planning_group.allow_replanning(True)

	pose_target = geometry_msgs.msg.Pose()
	pose_target.position.x = x
	pose_target.position.y = y
	pose_target.position.z = z
	pose_target.orientation.w = 1

	display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', \
	moveit_msgs.msg.DisplayTrajectory)
	print "============ Waiting for RVIZ..."

	moveit_planning_group.set_pose_target(pose_target)
	plan=moveit_planning_group.plan()
	
	# If plan is not found
	if not plan.joint_trajectory.points:
		sys.exit("No Motion Plan Found.")
	
	print "============ Visualizing plan1"
	display_trajectory = moveit_msgs.msg.DisplayTrajectory()

	display_trajectory.trajectory_start = moveit_robot_commander.get_current_state()
	display_trajectory.trajectory.append(plan)
	display_trajectory_publisher.publish(display_trajectory);

	print "============ Waiting while plan1 is visualized (again)..."

	key_input = raw_input("Proceed? [y/n]")
	
	if key_input == 'y':
		print "motion go 1"
		moveit_planning_group.go()
	else:
		print "NO"
		sys.exit("Exit")

	moveit_commander.roscpp_shutdown()