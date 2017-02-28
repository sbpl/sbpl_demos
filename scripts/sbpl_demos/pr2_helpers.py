#! /usr/bin/env python
import sys
import roslib
import rospy
import actionlib
import math

from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from tf2_msgs.msg import LookupTransformAction, LookupTransformGoal
from pr2_controllers_msgs.msg import PointHeadGoal, PointHeadAction
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PointStamped
from sbpl_demos.msg import RoconMoveArmAction, RoconMoveArmGoal, RoconMoveArmResult
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal
from pr2_common_action_msgs.msg import TuckArmsAction, TuckArmsGoal

## for moveit commander
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler


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

class TorsoCommand:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('/torso_controller/position_joint_action', SingleJointPositionAction)
        print("waiting for server...")
        self.client.wait_for_server()

    def MoveTorso(self, height):
        goal = SingleJointPositionGoal()
        goal.position = height 
        self.client.send_goal(goal)
        if self.client.wait_for_result(rospy.Duration(3.0)):
            return True
        else:
            return False

class TuckArms:
    def __init__(self):
        self.client = actionlib.SimpleActionClient("/tuck_arms", TuckArmsAction)
        print ("Waiting for tuck arms server...")
        self.client.wait_for_server()
    def TuckArms(self):
        goal = TuckArmsGoal()
        goal.tuck_left = True
        goal.tuck_right = True
        self.client.send_goal(goal)
        rospy.sleep(5.0)

    def TuckLeftArm(self):
        goal = TuckArmsGoal()
        goal.tuck_left = True
        goal.tuck_right = False
        self.client.send_goal(goal)
        rospy.sleep(5.0)

    def TuckRightArm(self):
        goal = TuckArmsGoal()
        goal.tuck_left = False
        goal.tuck_right = True
        self.client.send_goal(goal)
        rospy.sleep(5.0)

    def UntuckArms(self):
        goal = TuckArmsGoal()
        goal.tuck_left = False
        goal.tuck_right = False
        self.client.send_goal(goal)
        rospy.sleep(5.0)



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
            if self.left_client.wait_for_result(rospy.Duration(3.0)):
                return True
            else:
                return False
        else:
            self.right_client.send_goal(goal)
            if self.right_client.wait_for_result(rospy.Duration(3.0)):
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
        if self.client.wait_for_result(rospy.Duration(15.0)):
            return True
        else:
            return False

    def MoveToInternDesk(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = -0.5
        pose.position.y = -1.8
        pose.position.z = 0.0
        quat = quaternion_from_euler(0,0,-math.pi/2.0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.MoveToPose("map", pose)

    def MoveToWorkstation(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.9
        pose.position.y = -0.9
        pose.position.z = 0.0
        quat = quaternion_from_euler(0,0,-math.pi/2.0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.MoveToPose("map", pose)

class MoveitMoveArm:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        self.moveit_robot_commander = moveit_commander.RobotCommander()
        self.moveit_planning_scene = moveit_commander.PlanningSceneInterface()
        
        self.moveit_planning_group = moveit_commander.MoveGroupCommander("right_arm")
        self.moveit_planning_group.set_planner_id("RRTkConfigDefault")
        self.moveit_planning_group.set_planning_time(10.0)
        self.moveit_planning_group.allow_replanning(True)

    def Cleanup(self):
        moveit_commander.roscpp_shutdown()

    def MoveToPose(self, pose, reference_frame):
        print "MoveitMoveArm is moving to pose"
        self.moveit_planning_group.set_pose_target(pose)
        self.moveit_planning_group.set_pose_reference_frame(reference_frame)
        plan=self.moveit_planning_group.plan()
        if not plan.joint_trajectory.points:
            return False
        self.moveit_planning_group.go(wait=True)
        return True


    def MoveToHandoff(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.62
        pose.position.y = 0
        pose.position.z = 0.93
        quat = quaternion_from_euler(math.pi/2,0,0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.MoveToPose(pose, "base_footprint")

    def MoveToHome(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.09
        pose.position.y = -0.75
        pose.position.z = 0.89
        quat = quaternion_from_euler(0,0,0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.MoveToPose(pose, "base_footprint")

    def MoveToWide(self):
        jointvals = [-0.40000593078694135, 
        0.9999434123395927, 
        -0.00011982897502016421, 
        -2.0499264800290113, 
        6.283186160291728, 
        -0.1000014256823869, 
        6.283201089884446]
        self.moveit_planning_group.set_joint_value_target(jointvals)
        plan=self.moveit_planning_group.plan()
        if not plan.joint_trajectory.points:
            return False
        self.moveit_planning_group.go(wait=True)
        return True

    def AddDeskCollisionObjects(self):
        intern_desk_pose = geometry_msgs.msg.PoseStamped()
        intern_desk_pose.pose.position.x = -0.75
        intern_desk_pose.pose.position.y = -3.05
        intern_desk_pose.pose.position.z = 0.35
        intern_desk_pose.pose.orientation.x = 0
        intern_desk_pose.pose.orientation.y = 0
        intern_desk_pose.pose.orientation.z = 0
        intern_desk_pose.pose.orientation.w = 1
        intern_desk_pose.header.stamp = rospy.Time.now()
        intern_desk_pose.header.frame_id = "/map"

        self.moveit_planning_scene.add_box("intern_desk", intern_desk_pose, size=(1.52, .67, .7) )

        workstation_desk_pose = geometry_msgs.msg.PoseStamped()
        workstation_desk_pose.pose.position.x = 1.65
        workstation_desk_pose.pose.position.y = -1.75
        workstation_desk_pose.pose.position.z = 0.35
        workstation_desk_pose.pose.orientation.x = 0
        workstation_desk_pose.pose.orientation.y = 0
        workstation_desk_pose.pose.orientation.z = 0
        workstation_desk_pose.pose.orientation.w = 1
        workstation_desk_pose.header.stamp = rospy.Time.now()
        workstation_desk_pose.header.frame_id = "/map"

        self.moveit_planning_scene.add_box("workstation_desk", workstation_desk_pose, size=(1.52, .67, .7) )


class RoconMoveArm:
    def __init__(self):
        self.client = actionlib.SimpleActionClient('rocon_move_arm', RoconMoveArmAction)
        self.client.wait_for_server()
        print "RoconMoveArm online"

    def MoveToHandoff(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.62
        pose.position.y = 0
        pose.position.z = 0.93
        quat = quaternion_from_euler(math.pi/2,0,0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.MoveToPose(pose)

    def MoveToHome(self):
        pose = geometry_msgs.msg.Pose()
        pose.position.x = 0.09
        pose.position.y = -0.94
        pose.position.z = 0.89
        quat = quaternion_from_euler(0,0,0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.MoveToPose(pose)

    def MoveToPose(self, pose):
        print "RoconMoveArm is moving to Pose"
        goal = RoconMoveArmGoal()
        goal.target_pose = pose
        self.client.send_goal(goal)
        if self.client.wait_for_result(rospy.Duration(15.0)):
            result = self.client.get_result()
            return result.success
        return False
