#! /usr/bin/env python
import sys
import roslib
roslib.load_manifest('sbpl_demos')
roslib.load_manifest('pr2_controllers_msgs')
import rospy
import actionlib
import math

import IPython
#IPython.embed()
from pr2_controllers_msgs.msg import *

def state_cb(msg):
    print msg.feedback.position

if __name__ == '__main__':
    rospy.init_node('client')
    #IPython.embed()
    client = actionlib.SimpleActionClient('/torso_controller/position_joint_action', SingleJointPositionAction)
    #sub = rospy.Subscriber("pr2/torso_controller/position_joint_action/feedback", SingleJointPositionFeedback, state_cb)
    print("waiting for server...")
    #pub = rospy.Publisher("/torso_controller/position_joint_action/goal", SingleJointPositionActionGoal, queue_size=1)
    client.wait_for_server()
    #print("connected!")
    #rospy.sleep(1)

    goal = SingleJointPositionGoal()
    goal.position = int(sys.argv[1]) 
    client.send_goal(goal)
    #pub.publish(goal)
    rospy.sleep(2.0)
    print ('sent goal!')

