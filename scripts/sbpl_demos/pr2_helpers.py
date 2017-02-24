#! /usr/bin/env python
import sys
import roslib
import rospy
import actionlib
import math

import IPython
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from tf2_msgs.msg import LookupTransformAction, LookupTransformGoal
from pr2_controllers_msgs.msg import PointHeadGoal, PointHeadAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

from geometry_msgs.msg import PointStamped


class TFLookup:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('tf2_buffer_server', LookupTransformAction)
        self.client.wait_for_server()
        print("TFLookup Action online")

    def getTransform(self, source_frame, target_frame):
        goal = LookupTransformGoal()
        goal.target_frame = target_frame
        goal.source_frame = source_frame
        goal.timeout = rospy.Time(1)
        self.client.send_goal(goal)
        if self.client.wait_for_result():
            res = self.client.get_result()
            return res.transform
        else:
            raise ValueError('tf2_buffer_server was not called correctly')
            return LookupTransformResult()

class PointHead:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
        self.client.wait_for_server()
        print("PointHead online")

    def LookAt(self, frame, x, y, z):
        goal=PointHeadGoal()
        point=PointStamped()
        point.point.x=x
        point.point.y=y
        point.point.z=z
        point.header.frame_id=frame
        
        goal.target=point
        goal.pointing_frame="high_def_frame"
        goal.min_duration=rospy.Duration(0.5)
        goal.max_velocity=1.0

        self.client.send_goal(goal)
        if self.client.wait_for_result(rospy.Duration(3.0)):
            return True 
        else:
            return False

class GripperCommand:
    def __init__(self):
        self.left_client = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', GripperCommandAction)
        self.right_client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', GripperCommandAction)
        self.left_client.wait_for_server()
        self.right_client.wait_for_server()
        print("GripperCommand online")

    def Command(self, gripper, open):

        goal = GripperCommandGoal()
        if open == 1:
            goal.command.position = 0.09
        else:
            goal.command.position = 0.0
        goal.command.max_effort = float(-1.0) 
        
        if gripper == 'l':
            self.left_client.send_goal(goal)
            if self.left_client.wait_for_result():
                return True
            else:
                return False
        else:
            self.right_client.send_goal(goal)
            if self.right_client.wait_for_result():
                return True
            else:
                return False

class MoveBase:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        print "MoveBase online"

    def MoveToPose(self, frame, input_pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = frame
        goal.target_pose.pose = input_pose

        self.client.send_goal(goal)
        if self.client.wait_for_result():
            return True
        else:
            return False

class RoconMoveArm:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('rocon_move_arm', RoconMoveArmAction)
        self.client.wait_for_server()
        print "RoconMoveArm online"

    def MoveToPose(self, pose):
        goal = RoconMoveArmGoal()
        goal.pose = pose
        self.client.send_goal(goal)
        if self.client.wait_for_result():
            result = self.client.get_result()
            return result.success
        return False