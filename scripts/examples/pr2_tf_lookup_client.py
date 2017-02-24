#! /usr/bin/env python
import sys
import roslib
roslib.load_manifest('sbpl_demos')
roslib.load_manifest('tf2_msgs')
import rospy
import actionlib
import math

from tf2_msgs.msg import LookupTransformAction, LookupTransformGoal

if __name__ == '__main__':
    rospy.init_node('client')

    client = actionlib.SimpleActionClient('tf2_buffer_server', LookupTransformAction)
    print("waiting for server...")

    client.wait_for_server()

    goal = LookupTransformGoal()
    '''
    #Simple API
    string target_frame
    string source_frame
    time source_time
    duration timeout

    #Advanced API
    time target_time
    string fixed_frame

    #Whether or not to use the advanced API
    bool advanced

    ---
    geometry_msgs/TransformStamped transform
    tf2_msgs/TF2Error error
    ---
    '''
    goal.target_frame = sys.argv[1]
    goal.source_frame = sys.argv[2]
    #goal.source_time = rospy.Time.now()
    goal.timeout = rospy.Time(1)
    client.send_goal(goal)
    if client.wait_for_result():
        res = client.get_result()
        print res.transform
    else:
        print "Did not complete successfully"
    print ('Done!')

