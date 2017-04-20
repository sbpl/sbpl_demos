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


from sbpl_demos.srv import StateMachine, StateMachineRequest

class RomanMove:
    def __init__(self):
        self.perch = PerchClient()
        self.tflistener = tf.TransformListener()
        self.tfbroadcaster = tf.TransformBroadcaster()
        self.RMAC = RomanMoveArm()
        self.RCG = RomanCommandGripper()
        self.db = None
        self.loadGraspDatabase("roman_grasp_db.json")
        self.res = ""

    def roconclient_getobject(self):
        print "waiting for object request"

        while not rospy.is_shutdown():
            StateMachineRequest.command = "Get"
            StateMachineReqeust.request_key = "requested_object"
            StateMachineRequest.request_value = ""
            self.res = StateMachineClient(StateMachineRequest())

            if (res.result_value != ""):
                objresult = res.result_value
                break

            else:
                rospy.sleep(1)
        print objresult, " requested"

    def roconclient_send_complete(self):
        print "sending ROMAN_STATE to rocon"
        StateMachineRequest.command = "Set"
        StateMachineRequest.request_key = "ROMAN_STATE"
        StateMachineRequest.request_value = "DONE"
        self.res = StateMachineClient(StateMachineRequest())

    def loadGraspDatabase(self, db_file):
        with open(db_file,'r') as f:
            serialized = json.load(f)
            self.db = { key : [ { state : np.array(value) for state,value in grasp.iteritems() } for grasp in grasps ] for key, grasps in serialized.iteritems()}


    def moveToGrasp(self,T_w_p):
        pose = Pose()
        pose.position.x = 0.9174
        pose.position.y = -0.2263
        pose.position.z = 0.4823
        pose.orientation.w = 0.7576
        pose.orientation.x = 0.4476
        pose.orientation.y = 0.4109
        pose.orientation.z = 0.2385
        (pos, quat) = self.computeGraspPose("006_mustard_bottle", T_w_p)
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        print pose
        self.RMAC.MoveToPose(pose, "base_footprint")


    def moveToPreGrasp(self, T_w_p):
        pose = Pose()
        (pos, quat) = self.computePreGraspPose("006_mustard_bottle", T_w_p)
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        print pose
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
        pose.position.z = 0.96672
        pose.orientation.w = 0.596323
        pose.orientation.x = 0.608277
        pose.orientation.y = -0.23037
        pose.orientation.z = -0.47045
        self.RMAC.MoveToPose(pose, "base_footprint")

    def computeGraspPose(self, obj, T_w_p):
        transforms = self.computeEEPosesFromDatabase(obj,T_w_p)
        T_w_g = transforms[0]["grasp"]
        print T_w_g
        return (T_w_g[:3,3], tr.quaternion_from_matrix(T_w_g)) 

    def computePreGraspPose(self, obj, T_w_p):
        transforms = self.computeEEPosesFromDatabase(obj,T_w_p)
        T_w_g = transforms[0]["grasp"]
        print T_w_g
        pregrasp = np.array([[1,0,0, -.2], [0, 1, 0, 0], [0,0,1,0], [0,0,0,1]])
        pregrasp_tform = np.dot(T_w_g,pregrasp)
        return (pregrasp_tform[:3,3], tr.quaternion_from_matrix(pregrasp_tform)) 


    def computeEEPosesFromDatabase(self, obj, T_w_p):
        if(not self.db.has_key(obj)):
            print obj+" does not exist in grasp database!"
            return (None, None)
        grasps = self.db[obj]
        print grasps
        return [ { key : np.dot(T_w_p, tform) for key,tform in grasp.iteritems()  } for grasp in grasps]


    def computeTransformWithPerch(self, obj):
        object_pose = self.perch.getObjectPose(obj)
        print object_pose
        object_transf_trans = (object_pose.position.x, object_pose.position.y, object_pose.position.z)
        object_transf_rot = (object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w)
        T_w_p = self.tflistener.fromTranslationRotation(object_transf_trans, object_transf_rot)
        return T_w_p



if __name__ == "__main__":

    rospy.init_node('roman_move_to_home')
    test = RomanMove()
    #test.roconclient_getobject()

    T_w_p = test.computeTransformWithPerch("006_mustard_bottle")
    print T_w_p
    pregrasp = test.computePreGraspPose("006_mustard_bottle",T_w_p)
    print pregrasp
    
    #test.RMAC.MoveToHome()
    #test.RCG.Open()
    test.moveToPreGrasp(T_w_p)
    test.moveToGrasp(T_w_p)
    #test.RCG.Close()
    #test.moveToPreGrasp(T_w_p)
    #test.moveToPreTable()
    #test.moveToTable()
    #test.RCG.Open()
    #test.moveToPreTable()
    #test.RMAC.MoveToHome()

    #test.roconclient_send_complete()
    
    rospy.signal_shutdown("Shutdown Cleanly")
    rospy.spin()
