#! /usr/bin/env python
import sys
import rospy
import actionlib
import tf
import math
from tf.transformations import quaternion_from_euler
from sbpl_demos.roman_helpers import RomanMoveArm, RomanCommandGripper
import sbpl_demos.grasping_helpers as grasping_helpers
import sbpl_demos.perception_helpers as perception_helpers
from sbpl_demos.perception_helpers import AR_TYPES
from geometry_msgs.msg import Pose, PoseStamped
import moveit_commander
from sbpl_demos.perch_helpers import PerchClient


class RomanMove:
    def __init__(self):
        self.perch = PerchClient()
        self.tflistener = tf.TransformListener()
        self.tfbroadcaster = tf.TransformBroadcaster()
        self.RMAC = RomanMoveArm() 
        self.RCG = RomanCommandGripper()
       # 

	#self.addtable.addCylinder('roundtable',0.3,0.5,0.468,-0.767,0.3)
        #self.RomanARGrasping = grasping_helpers.RomanARGrasping()
        #self.ARTagListener = perception_helpers.ARTagListener()

    

    def moveToGrasp(self):
        pose = Pose()
        #pose.position.x = 0.8601
        #pose.position.y = -0.304
        #pose.position.z = 0.24126
        #pose.orientation.x = 0.424
        #pose.orientation.y = 0.48864
        #pose.orientation.z = 0.34858
        #pose.orientation.w = 0.6782
        pose.position.x = 0.9174
        pose.position.y = -0.2263
        pose.position.z = 0.4823
        pose.orientation.w = 0.7576
        pose.orientation.x = 0.4476
        pose.orientation.y = 0.4109
        pose.orientation.z = 0.2385
        self.RMAC.MoveToPose(pose, "base_footprint")

    def moveToPreGrasp(self):
        pose = Pose()
        #pose.position.x = 0.8042
        #pose.position.y = -0.4814
        #pose.position.z = 0.3147
        #pose.orientation.x = 0.424
        #pose.orientation.y = 0.48864
        #pose.orientation.z = 0.34858
        #pose.orientation.w = 0.6782
        pose.position.x = 0.8077
        pose.position.y = -0.3721
        #pose.position.z = 0.5642
	pose.position.z = 1.7901
        pose.orientation.w = 0.7576
        pose.orientation.x = 0.4476
        pose.orientation.y = 0.4109
        pose.orientation.z = 0.2385
        self.RMAC.MoveToPose(pose, "base_footprint")

    def moveToTable(self):
        pose = Pose()
        pose.position.x = 0.45613
        pose.position.y = -0.99926
        pose.position.z = 0.9072
        pose.orientation.w = 0.596323
        pose.orientation.x = 0.608277
        pose.orientation.y = -0.230372
        pose.orientation.z = -0.470453
        self.RMAC.MoveToPose(pose, "base_footprint")
 
    def moveToPreTable(self):
        pose = Pose()
        pose.position.x = 0.36589
        pose.position.y = -0.830991
        #pose.position.z = 0.96672
	pose.position.z = 1.96672

        pose.orientation.w = 0.596323
        pose.orientation.x = 0.608277
        pose.orientation.y = -0.23037
        pose.orientation.z = -0.47045
        self.RMAC.MoveToPose(pose, "base_footprint")

if __name__ == "__main__":
    rospy.init_node('roman_move_to_home')
    test = RomanMove()
    #test.RMAC.addcollisiontable()
   
    #test.RMAC.MoveToHome()
    #test.RCG.Open()
    #test.moveToPreGrasp()
    #test.moveToGrasp()
    #test.RCG.Close()
    test.moveToPreGrasp()
    test.moveToPreTable()
    #test.moveToTable()
    #test.RCG.Open()
    #test.moveToPreTable()
    #test.RMAC.MoveToHome()"""
