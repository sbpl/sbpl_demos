#!/usr/bin/env python
import rospy
import tf
# from tf_lookup_server.srv import parent_frame,child_frame
from std_msgs.msg import String,Float64
from sbpl_demos.srv import TfLookup

def handle_tf_lookup_server(req):
	print req.parent_frame
	print req.child_frame



if __name__ == '__main__':
    rospy.init_node('tf_lookup_server')
    robot = rospy.get_param("~robot", "default_robot")

    service = rospy.Service(robot+'/tf_lookup_server', TfLookup, handle_tf_lookup_server)
    # service = rospy.Service('tf_lookup_server', parent_frame, handle_tf_lookup_server)
    
    rate=rospy.Rate(10)

    while not rospy.is_shutdown():
    	rate.sleep()

