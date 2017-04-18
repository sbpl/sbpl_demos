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
import yaml


#print out the positions, velocities, and efforts of the right arm joints
if __name__ == "__main__":
    rospy.init_node('objects_to_grasps')
    
    PerchClient = perch_helpers.PerchClient()
    
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        
        # object_pose = PerchClient.getGraspPose("003_cracker_box")
        #object_pose = PerchClient.getGraspPose("019_pitcher_base")
        object_pose = PerchClient.getGraspPose("006_mustard_bottle")
        # object_pose = PerchClient.getGraspPose("010_potted_meat_can") 

        for i in range(0,3):
            raw_input("hit enter to get pose")
            listener.waitForTransform("limb_right_link7", "base_footprint", rospy.Time(),rospy.Duration(5))

            try:
                (pose_base_to_wrist, quat_base_to_wrist) = listener.lookupTransform("limb_right_link7", "base_footprint", rospy.Time())
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
            object_transf_rot = (object_pose.orientation.w, object_pose.orientation.x, object_pose.orientation.y, object_pose.orientation.z)

            T_b_w = listener.fromTranslationRotation(pose_base_to_wrist, quat_base_to_wrist)
            T_b_o = listener.fromTranslationRotation(object_transf_trans, object_transf_rot)
            
            T_o_b = np.linalg.inv(T_b_o)

            print "transforming..." + str(T_o_b )+ " times "+ str(T_b_w)

            T_o_w = T_o_b*T_b_w
            
            test = T_b_o*np.linalg.inv(T_o_w)
            print str(test)
            print str(T_o_b)
            
            quat = tr.quaternion_from_matrix(T_o_w)
            translation = T_o_w[:3,3]
            print "translation: " + str(translation)
            print "quat: " + str(quat)
            
            

