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
import json
import numpy as np
from tf import transformations as tr
class RomanMove:
    def __init__(self):
        self.perch = PerchClient()
        self.tflistener = tf.TransformListener()
        self.tfbroadcaster = tf.TransformBroadcaster()
        self.RMAC = RomanMoveArm()
        self.RCG = RomanCommandGripper()
        self.db = None
        self.loadGraspDatabase("roman_grasp_db.json")
        #self.addtable = moveit_commander.PlanningSceneInterface()

	#self.addtable.addCylinder('roundtable',0.3,0.5,0.468,-0.767,0.3)


    def loadGraspDatabase(self, db_file):
        with open(db_file,'r') as f:
            serialized = json.load(f)
            self.db = { key : [ { state : np.array(value) for state,value in grasp.iteritems() } for grasp in grasps ] for key, grasps in serialized.iteritems()}

    def addtable(self):
    	p = PoseStamped()
    	p.pose.position.x = 0.531
    	p.pose.position.y = -0.767
    	p.pose.position.z = 0.74
    	quat = quaternion_from_euler(-math.pi/2,0,0)
    	p.pose.orientation.x = quat[0]
    	p.pose.orientation.y = quat[1]
    	p.pose.orientation.z = quat[2]
    	p.pose.orientation.w = quat[3]
    	self.addtable.add_box("table", p, (0.9,0.9,0.74))

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
        pose.position.z = 0.5642
        pose.orientation.w = 0.7576
        pose.orientation.x = 0.4476
        pose.orientation.y = 0.4109
        pose.orientation.z = 0.2385
        self.RMAC.MoveToPose(pose, "base_footprint")


    def computePreGraspPose(self, obj, T_w_p):
        transforms = self.computeEEPosesFromDatabase(obj,T_w_p)
        T_w_g = transforms[0]["grasp"]
        print T_w_g
        pregrasp = np.array([[1,0,0, -.2], [0, 1, 0, 0], [0,0,1,0], [0,0,0,1]])
        pregrasp_tform = T_w_g*pregrasp
        return (pregrasp_tform[:3,3], tr.quaternion_from_matrix(pregrasp_tform)) 

    def computeEEPosesFromDatabase(self, obj, T_w_p):
        if(not self.db.has_key(obj)):
            print obj+" does not exist in grasp database!"
            return (None, None)
        grasps = self.db[obj]
        print grasps
        return [ { key : T_w_p * tform for key,tform in grasp.iteritems()  } for grasp in grasps]

    def computeTransformWithPerch(self, obj):
        object_pose = self.perch.getObjectPose(obj)
        object_transf_trans = (object_pose.position.x, object_pose.position.y, object_pose.position.z)
        object_transf_rot = (object_pose.orientation.w, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z)
        T_w_p = self.tflistener.fromTranslationRotation(object_transf_trans, object_transf_rot)
        return T_w_p



if __name__ == "__main__":
    rospy.init_node('roman_move_to_home')
    test = RomanMove()
    T_w_p = test.computeTransformWithPerch("006_mustard_bottle")
    print T_w_p
    pregrasp = test.computePreGraspPose("006_mustard_bottle",T_w_p)
    print pregrasp

    #test.RMAC.MoveToHome()
    #test.RCG.Open()
    ##test.RCG.Pinch()
    #test.RCG.Close()
    #test.RMAC.MoveToHandoff()
    #test.RCG.Open()
    #test.RCG.Close()
    #print("Now moving to map offset :")
    #test.addtable()
    #test.RMAC.MoveToHome()
    #test.RCG.Open()
    #test.moveToPreGrasp()
    #test.moveToGrasp()
    #test.RCG.Close()
    #while not rospy.is_shutdown():
    #   a = 1
    #test.moveToPreGrasp()
    #test.RMAC.MoveToHandoff()
    #test.RCG.Open()
    rospy.signal_shutdown("Shutdown Cleanly")
    rospy.spin()
