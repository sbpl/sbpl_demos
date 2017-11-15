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
from rospkg import RosPack
import os
from sbpl_demos.srv import StateMachine, StateMachineRequest
pkg = RosPack()
db_file = os.path.join(pkg.get_path("sbpl_demos"), "data/grasp_database/roman_grasp_db.json")

obj = "010_potted_meat_can"
#obj = "006_mustard_bottle"

class RomanMove:
    def __init__(self):
        self.perch = PerchClient()
        self.tflistener = tf.TransformListener()
        self.tfbroadcaster = tf.TransformBroadcaster()
        self.RMAC = RomanMoveArm()
        self.RCG = RomanCommandGripper()
        self.db = None
        self.db = self.loadGraspDatabase()
        self.res = ""

        '''
        print "waiting for state machine server..."
        rosopy.wait_for_service('state_machine')
        print "connected"
        self.StateMachineRequest = rospy.ServiceProxy('state_machine', StateMachine)
        self.StateMachineRequest = StateMachineRequest()
        '''


    def roconclient_getobject(self):
        print "waiting for object request"

        while not rospy.is_shutdown():
            self.StateMachineRequest.command = "Get"
            self.StateMachineReqeust.request_key = "requested_object"
            self.StateMachineRequest.request_value = ""
            self.res = StateMachineClient(StateMachineRequest())

            if (res.result_value != ""):
                objresult = res.result_value 
                print objresult, " requested"
                break

            else:
                rospy.sleep(1)

    def roconclient_send_busy(self):
        print "sending busy roman state to rocon"
        StateMachineRequest.command = "Set"
        StateMachineRequest.request_key = "ROMAN_STATE"
        StateMachineRequest.request_value = "BUSY"
        self.res = StateMachineClient(StateMachineRequest())


    def roconclient_send_complete(self):
        print "sending done roman state to rocon"
        StateMachineRequest.command = "Set"
        StateMachineRequest.request_key = "ROMAN_STATE"
        StateMachineRequest.request_value = "DONE"
        self.res = StateMachineClient(StateMachineRequest())

    def graspDatabaseToJSON(db):
        db = { key : [ { state : value.tolist() for state,value in grasp.iteritems() } for grasp in grasps ] for key, grasps in db.iteritems()}
        return json.dumps(db, indent=4,sort_keys=True, separators=(',',': '))

    def loadGraspDatabase(self):
        if not os.path.exists(db_file):
            print "No db file detected. Creating a new one"
            return dict()
        with open(db_file,'r') as f:
            try:
                serialized = json.load(f)
            except:
                print("Unable to deserialize json (creating new dict):", sys.exc_info()[0])
                return dict()
            try:
                db = { key : [ { state : np.array(value) for state,value in grasp.iteritems() } for grasp in grasps ] for key, grasps in serialized.iteritems()}
            except:
                print("Unable to parse json (creating new dict):", sys.exc_info()[0])
                return dict()
            return db


    def moveToGrasp(self,T_w_p):
        pose = Pose()
        '''
        pose.position.x = 0.9174
        pose.position.y = -0.2263
        pose.position.z = 0.4823
        pose.orientation.w = 0.7576
        pose.orientation.x = 0.4476
        pose.orientation.y = 0.4109
        pose.orientation.z = 0.2385
        '''
        (pos, quat) = self.computeGraspPose(obj, T_w_p)
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        state=self.RMAC.GenerateIK(pose,reference_frame="base_footprint")
        self.RMAC.VisualizeState(self.RMAC.visualizer_pub, state)
        test.RMAC.MoveToPose(pose, "base_footprint")


    def moveToPreGrasp(self, T_w_p):
        pose = Pose()
        (pos, quat) = self.computePreGraspPose(obj, T_w_p)
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        print "PreGrasp"+str(pose)
        sol=self.RMAC.GenerateIK(pose,reference_frame="base_footprint")
        self.RMAC.VisualizeState(self.RMAC.visualizer_pub, sol.solution.joint_state)
        self.RMAC.MoveToPose(pose, "base_footprint")

    def moveToTable(self):
        adj_x = 0.04
        adj_y = -0.01
        pose = Pose()
        pose.position.x = 0.64353 + adj_x
        pose.position.y = -0.8327 + adj_y
        pose.position.z = 0.7388
        pose.orientation.w = 0.63323
        pose.orientation.x = -0.25537
        pose.orientation.y = 0.69372
        pose.orientation.z = 0.22924
        state=self.RMAC.GenerateIK(pose,reference_frame="base_footprint")
        self.RMAC.VisualizeState(self.RMAC.visualizer_pub, state.solution)
        test.RMAC.MoveToPose(pose, "base_footprint")
 

    def moveToPreTable(self):
        adj_x = 0.00
        adj_y = -0.00
        pose = Pose()
        pose.position.x = 0.64353 + adj_x
        pose.position.y = -0.8327 + adj_y
        pose.position.z = 0.7888
        pose.orientation.w = 0.63323
        pose.orientation.x = -0.25537
        pose.orientation.y = 0.69372
        pose.orientation.z = 0.22924
        state=self.RMAC.GenerateIK(pose,reference_frame="base_footprint")
        self.RMAC.VisualizeState(self.RMAC.visualizer_pub, state.solution)
        self.RMAC.MoveToPose(pose, "base_footprint")

    def computeGraspPose(self, obj, T_w_p):
        transforms = self.computeEEPosesFromDatabase(obj,T_w_p)
        T_w_g = transforms[0]["grasp"]
        print "T_w_g"+ str(T_w_g)
        return (T_w_g[:3,3], tr.quaternion_from_matrix(T_w_g)) 

    def computePreGraspPose(self, obj, T_w_p):
        transforms = self.computeEEPosesFromDatabase(obj,T_w_p)
        T_w_g = transforms[0]["grasp"]
        print "T_w_g"+ str(T_w_g)
        pregrasp = np.array([[1,0,0, -.14], [0, 1, 0, 0], [0,0,1,0], [0,0,0,1]])
        pregrasp_tform = np.dot(T_w_g,pregrasp)
        print "T_w_pg" + str(pregrasp_tform)
        return (pregrasp_tform[:3,3], tr.quaternion_from_matrix(pregrasp_tform)) 


    def computeEEPosesFromDatabase(self, obj, T_w_p):
        '''T_w_p = T_world_object (i.e. object's pose in the world frame)'''
        '''Returns candidates grasp poses in the world frame (whatever frame object is in)'''
        if(not self.db.has_key(obj)):
            print obj+" does not exist in grasp database!"
            return (None, None)
        grasps = self.db[obj]
        print "grasps" + str(grasps)
        return [ { key : np.dot(T_w_p, np.linalg.inv(tform)) for key,tform in grasp.iteritems()  } for grasp in grasps]


    def computeTransformWithPerch(self, obj):
        object_pose = self.perch.getObjectPose(obj)
        print "object pose from perch "+ str(object_pose)
        object_transf_trans = (object_pose.position.x, object_pose.position.y, object_pose.position.z)
        object_transf_rot = (object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w)
        T_w_p = self.tflistener.fromTranslationRotation(object_transf_trans, object_transf_rot)
        return T_w_p



if __name__ == "__main__":

    rospy.init_node('roman_move_to_home')
    test = RomanMove()
    #test.roconclient_getobject()
    #test.roconclient_send_busy() 

    print "Moving to home position"
    print "Skipping home position for PR2 safety"
    test.RMAC.moveToHome_jointgoal()
    #print "NEW HOME STATE:" + str(test.RMAC.jointstate)

    print "Opening gripper"
    test.RCG.Open()
    # when done with grasp
    #test.roconclient_send_complete()
    REDO = "Y"
    while REDO == "Y":
        print "Detecting object with perch"
        T_w_p = test.computeTransformWithPerch(obj)
        print "T_w_p"+str(T_w_p)
        REDO = raw_input("Enter Y to retry perch detection:")

    print "Moving to pregrasp location"
    test.moveToPreGrasp(T_w_p)
    raw_input("Next")
    print "Blindly moving to grasp location"
    test.moveToGrasp(T_w_p)
    raw_input("Next")

    print "Pinch that spam!"
    test.RCG.Pinch()
    print "Backing out..."
    test.moveToPreGrasp(T_w_p)
    raw_input("Next")
    print "Going to Table"
    test.moveToPreTable()
    raw_input("Next")
    #print "Getting closer"
    test.moveToTable()
    raw_input("Next")
    print "Open Gripper"
    test.RCG.Open()
    #print "back out"
    test.moveToPreTable()
    raw_input("Next")
    print "Go home"
    test.RMAC.moveToHome_jointgoal()
    #raw_input("Next")

    #test.roconclient_send_complete()
    '''
    rospy.signal_shutdown("Shutdown Cleanly")
    rospy.spin()
    '''