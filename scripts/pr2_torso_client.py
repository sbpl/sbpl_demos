#! /usr/bin/env python

import roslib
roslib.load_manifest('sbpl_demos')
roslib.load_manifest('pr2_controllers_msgs')
import rospy
import actionlib
import math

import IPython
#IPython.embed()
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal

if __name__ == '__main__':
    rospy.init_node('client')
    #IPython.embed()
    client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction)
    #sub = rospy.Subscriber("torso_controller/state", JointState, state_cb)
    client.wait_for_server()

    goal = SingleJointPositionGoal()
    goal.position = 0 
    client.send_goal(goal)
    rospy.sleep(2.0)
    print ('sent goal!')
