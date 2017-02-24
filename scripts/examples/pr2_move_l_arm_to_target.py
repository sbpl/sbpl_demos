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
from tf.transformations import quaternion_from_euler

# designated orientation: x=0 y=0 z=0 w=1 
if __name__ == "__main__":
	if len(sys.argv) < 4:
		print "Usage: ./pr2_move_r_arm_to_target.py [x coordinate] [y cooridnate] [z coordinate] [roll angle] [pitch angle] [yaw angle]"
		sys.exit()

	pos_x=float(sys.argv[1])
	pos_y=float(sys.argv[2])
	pos_z=float(sys.argv[3])

	orientation_x=float(sys.argv[4])
	orientation_y=float(sys.argv[5])
	orientation_z=float(sys.argv[6])

	rospy.init_node("pr2_move_r_arm_to_target")
	
	# Initialize MoveIt.
	moveit_commander.roscpp_initialize(sys.argv)
	moveit_robot_commander = moveit_commander.RobotCommander()
	moveit_planning_scene = moveit_commander.PlanningSceneInterface()
	moveit_planning_group = moveit_commander.MoveGroupCommander("left_arm")
	moveit_planning_group.set_planner_id("RRTkConfigDefault")
	moveit_planning_group.set_planning_time(10.0)
	moveit_planning_group.allow_replanning(True)

	pose_target = geometry_msgs.msg.Pose()
	pose_target.position.x = pos_x
	pose_target.position.y = pos_y
	pose_target.position.z = pos_z
	pose_target.orientation.w = 1

	quat = quaternion_from_euler(orientation_x,orientation_y,orientation_z)
	pose_target.orientation.x = quat[0]
	pose_target.orientation.y = quat[1]
	pose_target.orientation.z = quat[2]
	pose_target.orientation.w = quat[3]
	
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