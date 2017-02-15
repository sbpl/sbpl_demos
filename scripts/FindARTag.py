#! /usr/bin/env python
import sys
import roslib
import rospy
import geometry_msgs.msg
import tf
from geometry_msgs.msg import *
from ar_track_alvar_msgs.msg import AlvarMarkers

# Precondition: The routine invoking FindARTag should initialize node.
# 				rospy.init_node()
class FindARTag:
	def setup(self,img):
		cam_image_topic=img
		# cam_info="/".join(cam_image_topic.split('/')[0:-1])+"/camera_info"
		cam_info="/kinect_head/rgb/camera_info"
		# rospy.set_param("ar_track_alvar/cam_image_topic", cam_image_topic)
		# rospy.set_param("ar_track_alvar/cam_info_topic", cam_info)
		rospy.set_param("ar_track_alvar/cam_image_topic", cam_image_topic)
		rospy.set_param("ar_track_alvar/cam_info_topic", cam_info)
		
		rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback_get_pose)
		self.pub=rospy.Publisher('ar_tag_on_odom',geometry_msgs.msg.PoseStamped,queue_size=10)
		self.data =None
		self.cb_flag = False
		self.num_try = 0

		self.transform_listener=tf.TransformListener()
		self.pose_stamped=PoseStamped()

	def __init__(self, img="/kinect_head/depth_registered/points"):
		self.setup(img)
		# self.set_ar_transform()

	def callback_get_pose(self,data):
		self.data=None 
		if len(data.markers) != 0:
			self.data=data.markers[0]
			self.pose_stamped=PoseStamped(self.data.header,self.data.pose.pose)
			self.cb_flag=True

	def getMarker(self):
		while self.cb_flag is False and self.num_try < 100:
			rospy.sleep(0.01)
			self.num_try=self.num_try+1

		if self.cb_flag is True:
			pose_in_odom=self.transform_listener.transformPose("odom_combined", \
						self.pose_stamped)
		
		elif self.cb_flag is False:
			return None

		self.data =None
		self.cb_flag = False

		return pose_in_odom

	# Function for displaying in TF..
	def set_ar_transform(self):
		rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self.callback_get_pose)
		transform_broadcaster=tf.TransformBroadcaster()
		rate=rospy.Rate(0.1)
		while not rospy.is_shutdown():
			if self.cb_flag == True:
				try:
					pose_in_odom=self.transform_listener.transformPose("odom_combined", \
						self.pose_stamped)
					self.pub.publish(pose_in_odom)
					print pose_in_odom
					transform_broadcaster.sendTransform((pose_in_odom.pose.position.x, \
						pose_in_odom.pose.position.y,pose_in_odom.pose.position.z), \
					(pose_in_odom.pose.orientation.x, pose_in_odom.pose.orientation.y, \
						pose_in_odom.pose.orientation.z, pose_in_odom.pose.orientation.w), \
					rospy.Time.now(),"ar_tag_grasp", "odom_combined")
					
				except (tf.Exception, tf.LookupException, tf.ConnectivityException):
					continue
			rate.sleep()
		rospy.spin()

if __name__ == '__main__':
	rospy.init_node('ar_tag')
	find_ar_tag=FindARTag()
	rospy.sleep(1)		# should wait for 1 sec for invoking callback function.	