#! /usr/bin/env python
import sys
import roslib
roslib.load_manifest('sbpl_demos')
roslib.load_manifest('control_msgs')
import rospy
import actionlib
import math

import IPython
from control_msgs.msg import GripperCommandAction, GripperCommandGoal

def state_cb(msg):
    print msg.feedback.position

if __name__ == '__main__':
    rospy.init_node('client')

    client = actionlib.SimpleActionClient(sys.argv[1] + '_gripper_controller/gripper_action', GripperCommandAction)
    print("waiting for server...")

    client.wait_for_server()

    goal = GripperCommandGoal()
    goal.command.position = float(sys.argv[2])
    goal.command.max_effort = float(-1.0) 
    client.send_goal(goal)
    #pub.publish(goal)
    rospy.sleep(2.0)
    print ('sent goal!')

