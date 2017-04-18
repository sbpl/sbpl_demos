#! /usr/bin/env python
import math
import sys
import roslib
import rospy
import sbpl_demos.pr2_helpers as pr2
import sbpl_demos.FindARTag

## for moveit commander
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import *
from tf.transformations import quaternion_from_euler

# Collsion avoidance is not included. That means you need to check trajectory in Rviz 
# and confirm to execute the motion.
if __name__ == "__main__":
    rospy.init_node("simple_grasping_demo")
    
    PointHead = pr2.PointHead()
    GripperCommand = pr2.GripperCommand()
    MoveBase = pr2.MoveBase()

    ar_tag = FindARTag.FindARTag()
    # Initialize MoveIt.
    moveit_commander.roscpp_initialize(sys.argv)
    moveit_robot_commander = moveit_commander.RobotCommander()
    moveit_planning_scene = moveit_commander.PlanningSceneInterface()
    moveit_planning_group = moveit_commander.MoveGroupCommander("right_arm")
    moveit_planning_group.set_planner_id("RRTkConfigDefault")
    moveit_planning_group.set_planning_time(10.0)
    moveit_planning_group.allow_replanning(True)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', \
            moveit_msgs.msg.DisplayTrajectory)

    rospy.sleep(2)
    pose_ar_tag = ar_tag.getMarker()

    if pose_ar_tag is not None:
        # Open gripper.
        GripperCommand.Command('r', 1)
        pose_gripper = geometry_msgs.msg.Pose()
        pose_gripper.position = pose_ar_tag.pose.position
        pose_gripper.position.x = pose_gripper.position.x 
        pose_gripper.position.y = pose_gripper.position.y - 0.3
        pose_gripper.position.z = pose_gripper.position.z

        quat = quaternion_from_euler(0,0,math.pi/2)
        pose_gripper.orientation.x = quat[0]
        pose_gripper.orientation.y = quat[1]
        pose_gripper.orientation.z = quat[2]
        pose_gripper.orientation.w = quat[3]

        moveit_planning_group.set_pose_target(pose_gripper)
        plan=moveit_planning_group.plan()

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

        pose_gripper.position.y=pose_gripper.position.y+0.15
        pose_gripper.orientation.x=quat[0]
        pose_gripper.orientation.y=quat[1]
        pose_gripper.orientation.z=quat[2]
        pose_gripper.orientation.w=quat[3]
        moveit_planning_group.set_pose_target(pose_gripper)
        plan=moveit_planning_group.plan()
        
        print "============ Visualizing plan1"
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        display_trajectory.trajectory_start = moveit_robot_commander.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory);

        print "============ Waiting while plan1 is visualized (again)..."

        key_input = 0
        key_input = raw_input("Proceed? [y/n]")
        
        if key_input == 'y':
            print "motion go 2"
            moveit_planning_group.go()
            # Close gripper.
            GripperCommand.Command('r', 0)
        else:
            print "NO"
            sys.exit("Exit")

        pose_gripper = geometry_msgs.msg.Pose()
        pose_gripper.position.x = -1.5 
        pose_gripper.position.y = 2.0
        pose_gripper.position.z = 1.0
        pose_gripper.orientation.w = 1.0

        moveit_planning_group.set_pose_target(pose_gripper)
        plan=moveit_planning_group.plan()
        
        print "============ Visualizing plan1"
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()

        display_trajectory.trajectory_start = moveit_robot_commander.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory);

        print "============ Waiting while plan1 is visualized (again)..."

        key_input = 0
        key_input = raw_input("Proceed? [y/n]")
        
        if key_input == 'y':
            print "motion go 3"
            moveit_planning_group.go()

        else:
            print "NO"
            sys.exit("Exit")

    else:
        print "Can't detect AR Tag."

    print "Demonstration finished"
    moveit_commander.roscpp_shutdown()
