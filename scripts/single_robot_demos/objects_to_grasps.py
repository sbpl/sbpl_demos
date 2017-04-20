#!/usr/bin/env python
#test client for joint_states_listener

import roslib
roslib.load_manifest('sbpl_demos')
import rospy
from sbpl_demos.srv import ReturnJointStates
from geometry_msgs.msg import Pose
import tf
import time
import sys
import numpy as np
from sbpl_demos import perch_helpers
from tf import transformations as tr

def call_return_joint_states(joint_names):
    rospy.wait_for_service("return_joint_states")
    try:
        s = rospy.ServiceProxy("return_joint_states", ReturnJointStates)
        resp = s(joint_names)
    except rospy.ServiceException, e:
        print "error when calling return_joint_states: %s"%e
        sys.exit(1)
    for (ind, joint_name) in enumerate(joint_names):
        if(not resp.found[ind]):
            print "joint %s not found!"%joint_name
    return (resp.position, resp.velocity, resp.effort)


#pretty-print list to string
def pplist(list):
    return ' '.join(['%2.3f'%x for x in list])


#print out the positions, velocities, and efforts of the right arm joints
if __name__ == "__main__":
    rospy.init_node('objects_to_grasps')
    
    PerchClient = perch_helpers.PerchClient()
    
    listener = tf.TransformListener()
    
    rate = rospy.Rate(10.0)

    # object_pose = PerchClient.getObjectPose("003_cracker_box")
    # object_pose = PerchClient.getObjectPose("019_pitcher_base")
    object_pose = PerchClient.getObjectPose("010_potted_meat_can")
    
    # object_pose = PerchClient.getObjectPose("006_mustard_bottle")
    
    raw_input("object pose registered, hit enter to get first pose")
    
    while not rospy.is_shutdown():
        
        try:
            (pose_base_to_wrist, quat_base_to_wrist) = listener.lookupTransform("base_footprint", "r_wrist_roll_link", rospy.Time(0))
        except (tf.LookupException):
            print "lookup exception"
            continue
        except (tf.ConnectivityException):
            print "connect exception"
            continue
        except (tf.ExtrapolationException):
            print "extrap exception"
            continue
        # object_pose = Pose()
        # object_pose.position.x = 0
        # object_pose.position.y = 1
        # object_pose.position.z = 10

        # object_pose.orientation.x = 1 
        # object_pose.orientation.y = 1 
        # object_pose.orientation.z = 1 
        # object_pose.orientation.w = 1



        object_transf_trans = (object_pose.position.x, object_pose.position.y, object_pose.position.z)
        object_transf_rot = (object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z, object_pose.orientation.w)

        T_b_w = listener.fromTranslationRotation(pose_base_to_wrist, quat_base_to_wrist)
        T_b_o = listener.fromTranslationRotation(object_transf_trans, object_transf_rot)

        

        # T_o_b = np.eye(4)
        # T_o_b[:3, :3] = np.linalg.inv(T_b_o[:3,:3])
        # T_o_b[:3, 3] = -np.dot(T_o_b[:3, :3],T_b_o[:3,3])

        T_o_b = np.linalg.inv(T_b_o)

        print "transforming..." + str(T_o_b )+ " times "+ str(T_b_w)

        T_o_w = np.dot(T_o_b,T_b_w)

        #print tf.transformations.inverse_matrix(T_o_w[:3,:3])

        # print "Base to wrist: Pose: " + str(pose_base_to_wrist) + ", Quat:" + str(quat_base_to_wrist)
        # print "Wrist Transform: " + str(T_b_w)
        # print "Object Transform: " + str(T_b_o)
        # print "Object to Wrist Transform: \n" + str(T_o_w)
        # print "Object pose" + str(object_pose)
        print "quaternion: "+ str(tr.quaternion_from_matrix(T_o_w))
        print "translation vector: "+ str(T_o_w[:3,3])

        # (position, velocity, effort) = call_return_joint_states(joint_names)
        # print "position:", pplist(position)
        # print "velocity:", pplist(velocity)
        # print "effort:", pplist(effort)
        raw_input("hit enter to get next tf")


        # ####################################
        # ####################################
        # ####################################
        
        # try:
        #     (pose_base_to_wrist, quat_base_to_wrist) = listener.lookupTransform("base_footprint", "r_wrist_roll_link", rospy.Time(0))
        # except (tf.LookupException):
        #     print "lookup exception"
        #     continue
        # except (tf.ConnectivityException):
        #     print "connect exception"
        #     continue
        # except (tf.ExtrapolationException):
        #     print "extrap exception"
        #     continue

        # T_b_w = listener.fromTranslationRotation(pose_base_to_wrist, quat_base_to_wrist)
        # T_b_o = listener.fromTranslationRotation(object_transf_trans, object_transf_rot)

        

        # # T_o_b = np.eye(4)
        # # T_o_b[:3, :3] = np.linalg.inv(T_b_o[:3,:3])
        # # T_o_b[:3, 3] = -np.dot(T_o_b[:3, :3],T_b_o[:3,3])

        # T_o_b = np.linalg.inv(T_b_o)

        # print "transforming..." + str(T_o_b )+ " times "+ str(T_b_w)

        # T_o_w = np.dot(T_o_b,T_b_w)

        # #print tf.transformations.inverse_matrix(T_o_w[:3,:3])

        # # print "Base to wrist: Pose: " + str(pose_base_to_wrist) + ", Quat:" + str(quat_base_to_wrist)
        # # print "Wrist Transform: " + str(T_b_w)
        # # print "Object Transform: " + str(T_b_o)
        # # print "Object to Wrist Transform: \n" + str(T_o_w)
        # print "quaternion: "+ str(tr.quaternion_from_matrix(T_o_w))
        # print "translation vector: "+ str(T_o_w[:3,3])
        # # print "Object pose" + str(object_pose)

        # raw_input("hit enter to get next tf")
