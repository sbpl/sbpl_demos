#! /usr/bin/env python
import sys
import rospy
import roslib
import math
import actionlib
import tf
from tf.transformations import quaternion_from_euler
from rcta.msg import MoveArmGoal, MoveArmAction
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from control_msgs.msg import GripperCommandAction, GripperCommandGoal
from roman_client_ros_utils.msg import RobotiqSimpleCmd, RobotiqActivate, RobotiqGenericCmd,RomanState

import copy
import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import PositionIKRequest
from moveit_msgs.srv import GetPositionIK

class RomanMoveArm(object):
    def __init__(self):
        self.client=actionlib.SimpleActionClient('/move_arm', MoveArmAction)
        self.tflistener = tf.TransformListener()
        rospy.loginfo("Waiting for action /move_arm...")
        self.client.wait_for_server()
        rospy.loginfo("Connected to action /move_arm!")
        self.jointstate = None
        self.jointstate_sub = rospy.Subscriber("/joint_states", JointState, self.__jointstatecallback__ )
        self.romanstate_sub = rospy.Subscriber("/roman_state", RomanState, self.__romanstatecallback__ )
        self.addtable = moveit_commander.PlanningSceneInterface()
        self.ikproxy = rospy.ServiceProxy("/compute_ik", GetPositionIK) 
        pose = Pose()
        pose.position.x = 0.468
        pose.position.y = -0.767
        pose.position.z = 1.170
        quat = quaternion_from_euler(-math.pi/2,0,0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        self.HOME = pose
    def __romanstatecallback__(self, romanstate):
        self.romanstate = romanstate    

    def __jointstatecallback__(self, jointstate):
        self.jointstate = jointstate

    def MoveToHome(self):
        pose = Pose()
        pose.position.x = 0.468
        pose.position.y = -0.767
        pose.position.z = 1.170
        quat = quaternion_from_euler(-math.pi/2,0,0)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return self.MoveToPose(pose, "base_footprint")

    def addcollisiontable(self):
        p = PoseStamped()
        p.header.frame_id = "base_footprint"
        p.header.stamp = rospy.get_rostime()
        #p.pose.position.x = 0.531
        #p.pose.position.y = -0.767
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.position.z = 0.74
        quat = quaternion_from_euler(-math.pi/2,0,0)
        p.pose.orientation.x = quat[0]
        p.pose.orientation.y = quat[1]
        p.pose.orientation.z = quat[2]
        p.pose.orientation.w = quat[3]
        print("reached addtable")
        self.addtable.add_box("table", p, (0.9,0.9,0.37))

    def MoveToHandoff(self):
        pose = Pose()
        pose.position.x = 0.531
        pose.position.y = -0.682
        pose.position.z = 1.167
        quat = quaternion_from_euler(math.pi,0,-2.993)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return self.MoveToPose(pose, "base_footprint")

    def GenerateIK(self, desired_pose, avoid_collision=False, duration=.5, numAttempts = 10, reference_frame="map"):
        if(    not self.tflistener.frameExists(reference_frame) or not self.tflistener.frameExists("map")):
            rospy.logwarn("Warning: could not look up provided reference frame")
            return False
        input_ps = PoseStamped()
        input_ps.pose = desired_pose
        input_ps.header.frame_id = reference_frame
        correct_ps = self.tflistener.transformPose("map", input_ps )
        group_name = "right_arm_and_torso"
        tip_link = "limb_right_link7"
        ikreq = PositionIKRequest()
        ikreq.group_name = group_name
        ikreq.robot_state.joint_state = self.jointstate
        ikreq.avoid_collisions = False
        ikreq.ik_link_name = "limb_right_link7"
        ikreq.pose_stamped.pose = correct_ps.pose
        ikreq.timeout = rospy.Duration(duration)
        ikreq.attempts = numAttempts
        return self.ikproxy(ikreq)

    def MoveToPose(self, desired_pose, reference_frame="map"):
        goal=MoveArmGoal()
        '''
        # goal 
        uint8 EndEffectorGoal = 0
        uint8 JointGoal = 1
        uint8 CartesianGoal = 2
        uint8 type
        geometry_msgs/Pose goal_pose
        sensor_msgs/JointState goal_joint_state
        moveit_msgs/RobotState start_state
        octomap_msgs/Octomap octomap
        bool execute_path
        moveit_msgs/PlanningOptions planning_options
        '''

        goal.type = 0 # EEgoal

        if(    not self.tflistener.frameExists(reference_frame) or not self.tflistener.frameExists("map")):
            rospy.logwarn("Warning: could not look up provided reference frame")
            return False
        input_ps = PoseStamped()
        input_ps.pose = desired_pose
        input_ps.header.frame_id = reference_frame
        correct_ps = self.tflistener.transformPose("map", input_ps )

        goal.goal_pose = correct_ps.pose
        rospy.loginfo("sending robot to pose: ")
        print goal.goal_pose
        goal.execute_path = True

        rospy.loginfo("Sending goal to action Server")
        self.client.send_goal(goal)
        #result = self.client.wait_for_result()
        self.client.wait_for_result()
        result =  self.client.get_result()
        rospy.loginfo("Finished pose goal - result is %s", result.success)
        if result.success:
            return True
        else:
            return False

class RomanCommandGripper(object):
    def __init__(self):

        self.activate_pub = rospy.Publisher("roman_hand_activate", RobotiqActivate, queue_size=1)
        self.open_pub = rospy.Publisher("roman_hand_open", RobotiqSimpleCmd, queue_size=1)
        self.close_pub = rospy.Publisher("roman_hand_close", RobotiqSimpleCmd, queue_size=1)
        self.goto_pub = rospy.Publisher("roman_hand_goto", RobotiqGenericCmd, queue_size=1)
        
    def TheBird(self):
        goto_msg = RobotiqGenericCmd()
        goto_msg.mech = 0 # right
        goto_msg.cmd_positions[0] = 0
        goto_msg.cmd_positions[1] = 255
        goto_msg.cmd_positions[2] = 255
        goto_msg.cmd_positions[3] = 255
        goto_msg.cmd_speed[0] = 0
        goto_msg.cmd_speed[1] = 0
        goto_msg.cmd_speed[2] = 0
        goto_msg.cmd_speed[3] = 0
        goto_msg.cmd_force[0] = 0
        goto_msg.cmd_force[1] = 0
        goto_msg.cmd_force[2] = 0
        goto_msg.cmd_force[3] = 0
        self.goto_pub.publish(goto_msg)
        
    def GoTo(self, pos, speed, force):
        goto_msg = RobotiqGenericCmd()
        goto_msg.mech = 0 # right
        goto_msg.cmd_positions[0] = pos
        goto_msg.cmd_positions[1] = pos
        goto_msg.cmd_positions[2] = pos
        goto_msg.cmd_positions[3] = pos
        goto_msg.cmd_speed[0] = speed
        goto_msg.cmd_speed[1] = speed
        goto_msg.cmd_speed[2] = speed
        goto_msg.cmd_speed[3] = speed
        goto_msg.cmd_force[0] = force
        goto_msg.cmd_force[1] = force
        goto_msg.cmd_force[2] = force
        goto_msg.cmd_force[3] = force
        self.goto_pub.publish(goto_msg)

    def Activate(self):
        activate_msg = RobotiqActivate()
        activate_msg.mech = 0 # right
        activate_msg.activate = True
        self.activate_pub.publish(activate_msg)
        print "Commanding Robotiq Active, only need to do this once"
        rospy.sleep(1.0) 

    def Deactivate(self):
        activate_msg = RobotiqActivate()
        activate_msg.mech = 0 # right
        activate_msg.activate = False
        activate_pub.publish(activate_msg)
        print "Commanding Robotiq Deactivate"
        rospy.sleep(1.0) 

    def Close(self):
        #self.activate_pub.publish(self.activate_msg)
        goal=RobotiqSimpleCmd()
        goal.mech = 0 # right
        goal.close = True
        self.close_pub.publish(goal)
        print "Commanding Robotiq Close"
        rospy.sleep(3.0) 

    def Pinch(self):
        #self.activate_pub.publish(self.activate_msg)
        goal=RobotiqSimpleCmd()
        goal.mech = 0 # right
        goal.close = True
        goal.mode = 2
        self.close_pub.publish(goal)
        print "Commanding Robotiq Pinch"
        rospy.sleep(4.0) 

    def Wide(self):
        #self.activate_pub.publish(self.activate_msg)
        goal=RobotiqSimpleCmd()
        goal.mech = 0 # right
        goal.close = True
        goal.mode = 1
        self.close_pub.publish(goal)
        print "Commanding Robotiq Wide"
        rospy.sleep(3.0)

    def Open(self):
        #self.activate_pub.publish(self.activate_msg)
        goal=RobotiqSimpleCmd()
        goal.mech = 0 #right
        goal.close = False
        self.open_pub.publish(goal)
        self.open_pub.publish(goal)
        self.open_pub.publish(goal)
        print "Commanding Robotiq Open"
        rospy.sleep(3.0) 
