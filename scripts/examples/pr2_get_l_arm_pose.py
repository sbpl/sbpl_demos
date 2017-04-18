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

if __name__ == "__main__":
    rospy.init_node("pr2_get_r_arm_pose")
    
    # Initialize MoveIt.
    moveit_commander.roscpp_initialize(sys.argv)
    moveit_robot_commander = moveit_commander.RobotCommander()
    moveit_planning_scene = moveit_commander.PlanningSceneInterface()
    moveit_planning_group = moveit_commander.MoveGroupCommander("left_arm")
    moveit_planning_group.set_planner_id("RRTkConfigDefault")
    moveit_planning_group.allow_replanning(True)

    pose_current = moveit_planning_group.get_current_pose()
    moveit_commander.roscpp_shutdown()
    print pose_current
