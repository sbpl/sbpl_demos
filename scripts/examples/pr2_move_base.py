#! /usr/bin/env python
import sys
import roslib
roslib.load_manifest('sbpl_demos')
roslib.load_manifest('move_base_msgs')
import rospy
import actionlib
import math

import IPython
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

if __name__ == '__main__':
    rospy.init_node('client')

    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    print("waiting for server...")

    client.wait_for_server()
    #rospy.sleep(2)
    goal = MoveBaseGoal()
    '''
    geometry_msgs/PoseStamped target_pose
    ---
    ---
    geometry_msgs/PoseStamped base_position

    '''

    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.pose.position.x = .291
    goal.target_pose.pose.position.y = -1.334
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = 0
    goal.target_pose.pose.orientation.y = 0
    goal.target_pose.pose.orientation.z = -0.343
    goal.target_pose.pose.orientation.w = 0.94

    client.send_goal(goal)
    if client.wait_for_result():
        res = client.get_result()
        print res
        print "Moved to location successfully"
    else:
        print "Did not complete successfully"
    print ('Done!')

