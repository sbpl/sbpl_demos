#! /usr/bin/env python
import sys
import rospy
import actionlib
import tf
from sbpl_demos.roman_helpers import RomanMoveArm, RomanCommandGripper
import sbpl_demos.grasping_helpers as grasping_helpers
import sbpl_demos.perception_helpers as perception_helpers
from sbpl_demos.perception_helpers import AR_TYPES
from geometry_msgs.msg import Pose
from sbpl_demos.msg import RoconMoveArmAction, RoconMoveArmGoal, RoconMoveArmResult
from sbpl_demos.msg import RoconPickAction, RoconPickGoal, RoconPickResult
from sbpl_demos.msg import RoconRobotiqAction, RoconRobotiqGoal, RoconRobotiqResult



if __name__ == "__main__":
    rospy.init_node('roman_grasp_test_v3')
    PickClient=actionlib.SimpleActionClient('/roman_pick_action', RoconPickAction)
    ArmClient=actionlib.SimpleActionClient('/roman_move_arm', RoconMoveArmAction)
    GraspClient=actionlib.SimpleActionClient('/roman_grasp_action', RoconRobotiqAction)
    rospy.loginfo("Waiting for servers...")
    PickClient.wait_for_server()
    ArmClient.wait_for_server()
    GraspClient.wait_for_server()
    rospy.loginfo("Connected.")

    Home = RoconMoveArmGoal()
    Home.goal_type = RoconMoveArmGoal.HOME

    Handoff = RoconMoveArmGoal()
    Handoff.goal_type = RoconMoveArmGoal.HANDOFF

    Open = RoconRobotiqGoal()
    Open.grasp_type = RoconRobotiqGoal.OPEN 

    Close = RoconRobotiqGoal()
    Close.grasp_type = RoconRobotiqGoal.CLOSE 

    Pinch = RoconRobotiqGoal()
    Pinch.grasp_type = RoconRobotiqGoal.PINCH


    while not rospy.is_shutdown():
        # Start the demo
        print "go home"
        ArmClient.send_goal(Home)
        ArmClient.wait_for_result()

        GraspClient.send_goal(Open)
        GraspClient.wait_for_result()

        print "pick"
        PickClient.send_goal(RoconPickGoal())
        result = PickClient.wait_for_result()
        if result:
            ArmClient.send_goal(Handoff)
            ArmClient.wait_for_result()

            GraspClient.send_goal(Open)
            GraspClient.wait_for_result()
