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
from sbpl_demos import perch_helpers

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
    rospy.init_node('joint_state')
    
    PerchClient = perch_helpers.PerchClient()
    
    listener = tf.TransformListener()
    
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        
        # object_pose = PerchClient.getGraspPose("006_mustard_bottle")

        try:
            (pose_base_to_wrist, quat_base_to_wrist) = listener.lookupTransform("r_wrist_roll_link", "base_link", rospy.Time(0))
        except (tf.LookupException):
            print "lookup exception"
            continue
        except (tf.ConnectivityException):
            print "connect exception"
            continue
        except (tf.ExtrapolationException):
            print "extrap exception"
            continue
        object_pose = Pose()
        object_pose.position.x = 0
        object_pose.position.y = 1
        object_pose.position.z = 10

        object_pose.orientation.x = 1 
        object_pose.orientation.y = 1 
        object_pose.orientation.z = 1 
        object_pose.orientation.w = 1

        T_b_w = listener.fromTranslationRotation(pose_base_to_wrist, quat_base_to_wrist)
        T_b_o = listener.transformPose(object_pose)

        # T_o_w = inv(T_b_o)*T_b_w

        print "Base to wrist: Pose: " + str(pose_base_to_wrist) + ", Quat:" + str(quat_base_to_wrist)
        print "Wrist Transform: " + str(T_b_w)
        print "Object Transform: " + str(T_b_o)
        print "Object to Wrist Transform: " + str(T_o_w)
        print "Object pose" + str(object_pose)

        # (position, velocity, effort) = call_return_joint_states(joint_names)
        # print "position:", pplist(position)
        # print "velocity:", pplist(velocity)
        # print "effort:", pplist(effort)
        raw_input("hit enter to get next tf")
        