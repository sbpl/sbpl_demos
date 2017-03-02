#! /usr/bin/env python
import sys
import roslib
import rospy
from geometry_msgs.msg import PoseStamped
import tf
from ar_track_alvar_msgs.msg import AlvarMarkers
from sbpl_demos.srv import PoseUpsampleRequest, PoseUpsample
from sbpl_demos.msg import XYZRPY

class ARTagListener:
	def __init__(self):
		self.sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback_get_pose)
		self.latest_pose=PoseStamped()
		self.latest_marker=AlvarMarkers()
		self.last_reading = rospy.Time()

	def callback_get_pose(self,data):
		if len(data.markers) > 0:
			#simply update the latest pose with the first marker
			self.latest_pose=PoseStamped(data.markers[0].header, data.markers[0].pose.pose)
			self.latest_marker = data
			self.last_reading = rospy.Time.now()

