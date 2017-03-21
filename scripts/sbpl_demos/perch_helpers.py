#!/usr/bin/env python
import sys
import roslib
import rospy
import tf
import numpy

from std_msgs.msg import Float64MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import PointCloud2
from object_recognition_node.srv import LocalizeObjectsRequest, LocalizeObjects
from geometry_msgs.msg import TransformStamped
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PerchClient:
	def __init__(self):

		self.tflistener = tf.TransformListener()

		rospy.loginfo("Waiting for object_localizer_service...")
		rospy.wait_for_service('object_localizer_service')
		self.client = rospy.ServiceProxy('object_localizer_service', LocalizeObjects)
		rospy.loginfo("Connected.")

		self.request = LocalizeObjectsRequest()

# 		self.request.object_ids = ["011_banana", "019_pitcher_base", "024_bowl"]
		self.request.object_ids = ["011_banana"]

		self.request.x_min = 0.5
		self.request.x_max = 2.0
		self.request.y_min = -0.5
		self.request.y_max = 0.5
		self.request.support_surface_height = 0.725

		self.request.camera_pose = Float64MultiArray()
		self.request.input_organized_cloud = PointCloud2()
	
		self.locked = False

		rospy.Subscriber("/kinect_head/depth_registered/points", PointCloud2, self.kinect_callback)


	def kinect_callback(self, data):

		if not self.locked:

			print "callback!"
			self.locked = True

			self.input_cloud = data


	def send_request(self):

			# set camera_pose field
			# NOTE camera_pose is in body-frame convention (x-forward; head_mount_kinect_rgb_link), not in optical-frame convention (z-forward; head_mount_kinect_rgb_optical_frame)
			(camera_trans, camera_quat) = self.tflistener.lookupTransform("base_footprint", "head_mount_kinect_rgb_link", rospy.Time())
			camera_matrix = self.tflistener.fromTranslationRotation(camera_trans, camera_quat)

			camera_pose = Float64MultiArray()
			camera_pose_layout = MultiArrayLayout()
			camera_pose_dim = [MultiArrayDimension(), MultiArrayDimension()]
			camera_pose_dim[0].stride = 4*4
			camera_pose_dim[0].size = 4
			camera_pose_dim[1].stride = 4
			camera_pose_dim[1].size = 4
			camera_pose.layout.dim = camera_pose_dim
			camera_pose.data = numpy.ravel(camera_matrix)
			self.request.camera_pose = camera_pose


			# transform the input point cloud w.r.t /base_footprint
			(cloud_trans, cloud_quat) = self.tflistener.lookupTransform("head_mount_kinect_rgb_optical_frame", "base_footprint", rospy.Time())
			cloud_tf_stamped = TransformStamped()
			cloud_tf_stamped.header.stamp = rospy.Time.now()
			cloud_tf_stamped.header.frame_id = "base_footprint"		# this header will be used to set input_organized_cloud.header
			cloud_tf_stamped.child_frame_id = "base_footprint"
			cloud_tf_stamped.transform.translation.x = cloud_trans[0]
			cloud_tf_stamped.transform.translation.y = cloud_trans[1]
			cloud_tf_stamped.transform.translation.z = cloud_trans[2]
			cloud_tf_stamped.transform.rotation.x = cloud_quat[0]
			cloud_tf_stamped.transform.rotation.y = cloud_quat[1]
			cloud_tf_stamped.transform.rotation.z = cloud_quat[2]
			cloud_tf_stamped.transform.rotation.w = cloud_quat[3]
			self.request.input_organized_cloud = do_transform_cloud(self.input_cloud, cloud_tf_stamped)


 			# NOTE construct input_organized_cloud for LocalizeObjects.srv by
			# IT IS HIGHLY TRICKY TO MANIPULATE POINT CLOUD IN PYTHON...
# 			# setting the color of points outside of a bounding box [x_min:x_max, y_min:y_max, support_surface_height:] to black (r=g=b=0)
# 			for i in range(n_points):
# 				if self.request.input_organized_cloud.points[i].x < self.request.x_min:
# 					self.request.input_organized_cloud.points[i].r = 0
# 					self.request.input_organized_cloud.points[i].g = 0
# 					self.request.input_organized_cloud.points[i].b = 0


			res = self.client(self.request)
			print res


			# TODO make use of the object localization results for grasping



if __name__ == "__main__":
	rospy.init_node('object_localizer_client')
	perch_client = PerchClient()

	while not rospy.is_shutdown():
		print "in the loop.."

		if perch_client.locked:

			if not perch_client.send_request():
				rospy.logwarn("Object localization failed! Waiting for another observation...")

			self.locked = False

		rospy.sleep(1.0)

