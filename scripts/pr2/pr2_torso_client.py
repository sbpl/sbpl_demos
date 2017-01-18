#! /usr/bin/env python

import roslib
roslib.load_manifest('sbpl_demos')
import rospy
import actionlib
import math
from pr2_controllers_msgs.msg import SingleJointPositionAction, SingleJointPositionGoal

if __name__ == '__main__':
    rospy.init_node('client')
    import IPython
    #IPython.embed()
    client = actionlib.SimpleActionClient('pr2/torso_controller/position_joint_action', SingleJointPositionAction)
    #sub = rospy.Subscriber("torso_controller/state", JointState, state_cb)
    #client.wait_for_server()

    goal = SingleJointPositionGoal()
    goal.position = 1
    client.send_goal(goal)
    rospy.sleep(2.0)
    print ('sent goal!')
