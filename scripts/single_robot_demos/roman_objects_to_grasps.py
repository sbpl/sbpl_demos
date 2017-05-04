#!/usr/bin/env python
#test client for joint_states_listener

import roslib
roslib.load_manifest('sbpl_demos')
import rospy
from geometry_msgs.msg import Pose
import tf
import time
import sys
import numpy as np
from sbpl_demos import perch_helpers
from tf import transformations as tr
import json
from rospkg import RosPack
import os
import shutil
import time
pkg = RosPack()
db_file = os.path.join(pkg.get_path("sbpl_demos"), "data/grasp_database/roman_grasp_db.json")
def graspDatabaseToJSON(db):
    db = { key : [ { state : value.tolist() for state,value in grasp.iteritems() } for grasp in grasps ] for key, grasps in db.iteritems()}
    return json.dumps(db, indent=4,sort_keys=True, separators=(',',': '))
def loadGraspDatabase():
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
def save(db):
    with open(db_file, 'w') as f:
        f.write(graspDatabaseToJSON(db))
def backup(db):
    saveas = "/tmp/db.json.backup."+ time.strftime("%d.%m.%y.%H.%M.%S")
    with open(saveas, 'w') as f:
        f.write(graspDatabaseToJSON(db))
        return saveas
#print out the positions, velocities, and efforts of the right arm joints
if __name__ == "__main__":
    rospy.init_node('objects_to_grasps')
    PerchClient = perch_helpers.PerchClient()
    listener = tf.TransformListener()
    print "Loading Grasp Database..."
    db = loadGraspDatabase()
    print "OK!"
    while not rospy.is_shutdown():
        obj = raw_input("input object id: ")
        object_pose = PerchClient.getObjectPose(obj)
        for i in range(0,1):
            raw_input("hit enter to capture grasping pose for "+obj)
            listener.waitForTransform("limb_right_link7", "base_footprint", rospy.Time(),rospy.Duration(5))

            try:
                (pose_base_to_wrist, quat_base_to_wrist) = listener.lookupTransform("base_footprint", "limb_right_link7", rospy.Time())
            except (tf.LookupException):
                print "lookup exception"
                continue
            except (tf.ConnectivityException):
                print "connect exception"
                continue
            except (tf.ExtrapolationException):
                print "extrap exception"
                continue

            object_transf_trans = (object_pose.position.x, object_pose.position.y, object_pose.position.z)
            object_transf_rot = (object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w)

            T_b_w = listener.fromTranslationRotation(pose_base_to_wrist, quat_base_to_wrist)
            T_b_o = listener.fromTranslationRotation(object_transf_trans, object_transf_rot)
            T_o_b = np.linalg.inv(T_b_o)


            T_o_w = np.dot(T_o_b,T_b_w)
            T_w_o = np.linalg.inv(T_o_w)
            test = np.dot(T_b_o, T_o_w)
            if not np.allclose(test, T_b_w):
                print "Expected:" + str(test) + " to be close to " + T_b_w
            quat = tr.quaternion_from_matrix(T_w_o)
            translation = T_w_o[:3,3]
            print "translation: " + str(translation) + " quat: " + str(quat)
            option = raw_input("Save? (Y/N)")
            print "Backing up to..." + backup(db)
            if option == "Y" or option == "y":
                if not(obj in db):
                    db[obj]=list()
                db[obj].append({"grasp" : T_w_o}) 
            save(db)
            print "Updated Database!"
            
