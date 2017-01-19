#!/usr/bin/env python
import rospy
import tf
# from tf_lookup_server.srv import parent_frame,child_frame
from std_msgs.msg import String,Float64
from geometry_msgs.msg import Pose, Point, Quaternion
from sbpl_demos.srv import *

class TfLookupServer:
	def __init__(self):
		self.listener = tf.TransformListener()
		return
	def handle_tf_lookup_server(self, req):
		now = rospy.Time.now()
		self.listener.waitForTransform(req.parent_frame, req.child_frame, now, rospy.Duration(req.max_timeout))
		(trans,rot) = self.listener.lookupTransform(req.parent_frame, req.child_frame, now)

		pose = Pose()
		pose.position.x = trans[0]
		pose.position.y = trans[1]
		pose.position.z = trans[2]

		pose.orientation.x = rot[0]
		pose.orientation.y = rot[1]
		pose.orientation.z = rot[2]
		pose.orientation.w = rot[3]
		
		return TfLookupResponse(pose)

	def run(self):
		robot = rospy.get_param("robot", "default_robot")
		service = rospy.Service(robot+'/tf_lookup_server', TfLookup, self.handle_tf_lookup_server)		
		return

if __name__ == '__main__':
	rospy.init_node('tf_lookup_server')
	srv = TfLookupServer()
	srv.run()
	rospy.spin()

	rate=rospy.Rate(10)
	while not rospy.is_shutdown():
		rate.sleep()