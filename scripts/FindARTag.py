#! /usr/bin/env python
import sys
import roslib
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

class FindARTag:
	def setup(self,img):
		cam_image_topic=rospy.get_param("ar_track_alvar/cam_image_topic")
		
		cam_image_topic=img
		cam_info="/".join(cam_image_topic.split('/')[0:-1])+"/camera_info"

		rospy.set_param("ar_track_alvar/cam_image_topic", cam_image_topic)
		rospy.set_param("ar_track_alvar/cam_info_topic", cam_info)
		rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback_get_pose)

	def __init__(self, img="/narrow_stereo/left/image_color"):
		rospy.init_node('pr2_find_ar_tag')
		self.setup(img)

	def getMarker(self):
		if self.data == None:
			return None
		else:
			return self.data.pose
	
	def callback_get_pose(self,data):
		self.data=None
		if len(data.markers) != 0:
			self.data=data.markers[0]